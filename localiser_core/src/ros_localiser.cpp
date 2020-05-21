#include "ros_localiser.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>


int main(int argc, char** argv) {
/*
  LinearFilter linear_filter;
  linear_filter.state << 0., 0., 0.;

  linear_filter.state_var << 0.1, 0., 0.,
                             0., 0.1, 0.,
                             0., 0., 0.1;


  Eigen::Vector3d observation;
  observation << 1., 0., 0.;

  Eigen::Matrix3d observation_noise;
  observation_noise << 0.1, 0., 0.,
                       0., 0.1, 0.,
                       0., 0., 0.001;


  linear_filter.update(observation, observation_noise);

//  linear_filter.test_predict();
//  linear_filter.test_update();

  return 0;
*/

  // todo: parameters
  //  localise_input min speed to process update message

  ros::init(argc, argv, "localiser");
  ros::Time::init();

  ROSLocaliser ros_localiser;
  ros_localiser.Initialise();

  return 0;
}

// create the localiser modules
ROSLocaliser::ROSLocaliser() :
    localiser_input(std::make_shared<LocaliserInput>()),
    localiser_output(std::make_shared<LocaliserOutput>()),
    imu(std::make_shared<ImuMeasurement>()),
    speed(std::make_shared<SpeedMeasurement>()),
    map_icp(std::make_shared<ICPObservation>()),
    gnss(std::make_shared<GNSSObservation>()) {


  ros::NodeHandle n;
  service = n.advertiseService("instruct_localiser", &ROSLocaliser::InstructionCallback, this);
}


bool
ROSLocaliser::InstructionCallback(localiser::instruct_localiser::Request& req,
    localiser::instruct_localiser::Response& res) {

  /*# enum of instructionType
uint8 RESET=0
uint8 MAP_ON=1
uint8 MAP_OFF=2
uint8 ABS_ON=3
uint8 ABS_OFF=4
bool SUCCESS=True
bool FAILURE=False
##########################

uint8 instructionType
---
bool status
   */

  ROS_INFO_STREAM("received localiser instruction " << req.instructionType);
  res.status = true;

  return true;
}



void
ROSLocaliser::Initialise() {

  // bind update method
  map_icp->perform_update = std::bind(&LocaliserInput::Update, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  gnss->perform_update = std::bind(&LocaliserInput::Update, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  // bind set variables
  imu->update_pitch = std::bind(&LocaliserInput::SetPitch, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  imu->update_yaw_rate = std::bind(&LocaliserInput::SetYawRate, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  speed->update_speed = std::bind(&LocaliserInput::SetSpeed, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  // bind prediction trigger
  imu->perform_prediction = std::bind(&LocaliserInput::Predict, localiser_input, std::placeholders::_1);

  // output method
  // publish output or write to bag
  std::string output_bag;
  ros::param::param<std::string>("~output_bag", output_bag, "");

  if (output_bag.empty()) {
    ROS_INFO_STREAM("[OUTPUT] Publishing output");
    publisher = std::make_shared<Publisher>();
    localiser_output->publish_odom = std::bind(&Publisher::publish_odom, publisher, std::placeholders::_1, std::placeholders::_2);
    localiser_output->publish_fix = std::bind(&Publisher::publish_fix, publisher, std::placeholders::_1, std::placeholders::_2);
  }
  else {
    ROS_INFO_STREAM("[OUTPUT] Writing to bagfile " << output_bag);
    bag_output = std::make_shared<BagOutput>();
    bag_output->Initialise(output_bag);
    localiser_output->publish_odom = std::bind(&BagOutput::publish_odom, bag_output, std::placeholders::_1, std::placeholders::_2);
    localiser_output->publish_fix = std::bind(&BagOutput::publish_fix, bag_output, std::placeholders::_1, std::placeholders::_2);
  }

  // localisation method
  // run using gtsam or graph optimizer
  if (0) {
    /*
    // bind localisation method inputs
    gtsam_optimiser = std::make_shared<GtsamOptimiser>();
    localiser_input->perform_update = std::bind(&GtsamOptimiser::AddAbsolutePosition, gtsam_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    localiser_input->perform_prediction = std::bind(&GtsamOptimiser::AddRelativeMotion, gtsam_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // where to send localisation method outputs
    gtsam_optimiser->publish_odometry = std::bind(&LocaliserOutput::PublishOdometry, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    */
  }
  else if(0){
    // bind localisation method inputs
    graph_optimiser = std::make_shared<GraphOptimiser>();
    localiser_input->perform_update = std::bind(&GraphOptimiser::AddAbsolutePosition, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    localiser_input->perform_prediction = std::bind(&GraphOptimiser::AddRelativeMotion, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // where to send localisation method outputs
    graph_optimiser->publish_odometry = std::bind(&LocaliserOutput::PublishOdometry, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    graph_optimiser->publish_map = std::bind(&LocaliserOutput::PublishMap, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
  }
  else {
    linear_filter = std::make_shared<PositionOnlyEKF>();

    localiser_input->perform_update = std::bind(&LinearFilter::AddAbsolutePosition, linear_filter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    localiser_input->perform_prediction = std::bind(&LinearFilter::AddRelativeMotion, linear_filter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // where to send localisation method outputs
    linear_filter->publish_odometry = std::bind(&LocaliserOutput::PublishOdometry, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    linear_filter->publish_map = std::bind(&LocaliserOutput::PublishMap, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
  }


  // input method
  // from subscriber or rosbag play
  std::string input_bag;
  ros::param::param<std::string>("~input_bag", input_bag, "");

  if (input_bag.empty()) {
    ROS_INFO_STREAM("[INPUT] Subscribing to topics");
    // subscribe
    std::vector<ros::Subscriber> subscribers;

    ros::NodeHandle n;
    subscribers.push_back(n.subscribe("/vn100/imu", 1000, &ImuMeasurement::receive_message, &(*imu)));
    subscribers.push_back(n.subscribe("/ublox_gps/fix", 1000, &GNSSObservation::receive_message, &(*gnss)));
    subscribers.push_back(n.subscribe("/zio/odometry/rear", 1000, &SpeedMeasurement::receive_message, &(*speed)));
    subscribers.push_back(n.subscribe("/velodyne/front/poles/average", 1000, &ICPObservation::receive_message, &(*map_icp)));





//todo: make a list of topics for each type of message
//todo: in the localiser_input/output set the bind/unbind as required





    ros::spin();
  }
  else {
    ROS_INFO_STREAM("[INPUT] Reading data from bagfile" << input_bag);
    bag_input = std::make_shared<BagInput>();

    // odometry update messages (i.e. from ICP)
    // bag_input.odom_update_topics.insert("/localisation/gnss/utm")

    // GNSS fix messages
    bag_input->fix_update_topics.insert("/ublox_gps/fix");
    bag_input->fix_update_topics.insert("ibeo/gnss");

    // speed odometry messages
    bag_input->odom_speed_topics.insert("/zio/odometry/rear");
    bag_input->odom_speed_topics.insert("/vn100/odometry");
    bag_input->odom_speed_topics.insert("ibeo/odometry");

    // imu topics
    bag_input->imu_topics.insert("/vn100/imu");
    bag_input->imu_topics.insert("xsens/IMU");

    // imu topics
    bag_input->pointcloud_topics.insert("/velodyne/front/filtered");

    // bind bag input functions to appropriate message handlers
    bag_input->publish_odom_update = std::bind(&ICPObservation::receive_message, &(*map_icp), std::placeholders::_1);
    bag_input->publish_speed_update = std::bind(&SpeedMeasurement::receive_message, &(*speed), std::placeholders::_1);
    bag_input->publish_fix_update = std::bind(&GNSSObservation::receive_message, &(*gnss), std::placeholders::_1);
    bag_input->publish_imu_update = std::bind(&ImuMeasurement::receive_message, &(*imu), std::placeholders::_1);

    //features_pipeline = std::make_shared<PointCloudFeaturesPipeline>();
    //icp_pipeline = std::make_shared<ICPMatcherPipeline>();

    //bag_input->publish_pointcloud_update = std::bind(&PointCloudFeaturesPipeline::receive_message, &(*features_pipeline), std::placeholders::_1);
    //features_pipeline->publish_poles_corners = std::bind(&ICPMatcherPipeline::receive_message, &(*icp_pipeline), std::placeholders::_1, std::placeholders::_2);

    bag_input->ReadBag(input_bag);

  }
}

// todo: publish pointcloud from pipeline ? allow to make lidar_png. Work out how to use this with partial scans
//  (attach to graph ?, make separate subscans and put in time order with the tf tree buffer?)

// todo: make a launch for all other things required for pointcloudfeatures pipeline

// todo: publish transforms
