#include "ros_localiser.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <dataset_tools/LocaliserStats.h>

#include <vector>


int main(int argc, char** argv) {

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
    bag_input(std::make_shared<BagInput>()),
    bag_output(std::make_shared<BagOutput>()),
    publisher(std::make_shared<Publisher>()),
    localiser_input(std::make_shared<LocaliserInput>()),
    localiser_output(std::make_shared<LocaliserOutput>()),
    imu(std::make_shared<ImuMeasurement>()),
    speed(std::make_shared<SpeedMeasurement>()),
    map_icp(std::make_shared<ICPObservation>()),
    gnss(std::make_shared<GNSSObservation>()),
    graph_optimiser(std::make_shared<GraphOptimiser>()),
    linear_filter(std::make_shared<PositionHeadingEKF>()){


  ros::NodeHandle n;
  service = n.advertiseService("instruct_localiser", &ROSLocaliser::InstructionCallback, this);

  //! Read the parameters to determine how the localiser will work
  run_pipeline = false;
  ros::param::param<bool>("~run_pipeline", run_pipeline, false);

  // output method: publish output or write to bag
  ros::param::param<std::string>("~output_bag", output_bag, "");

  // input method: from subscriber or rosbag play
  ros::param::param<std::string>("~input_bag", input_bag, "");

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

  switch(req.instructionType) {
     case 0://RESET
        ROS_WARN_STREAM("RESET: localiser reset, reinitialising using GPS");
        gnss->perform_update = std::bind(&LocaliserInput::Update, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        map_icp->perform_update = nullptr;//std::function<void()>();

        // make a new object of localisation method
        linear_filter = std::make_shared<PositionHeadingEKF>();
        // connect localisation method inputs
        localiser_input->perform_update = std::bind(&LinearFilter::AddAbsolutePosition, linear_filter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        localiser_input->perform_prediction = std::bind(&LinearFilter::AddRelativeMotion, linear_filter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        // where to send localisation method outputs
        linear_filter->publish_odometry = std::bind(&LocaliserOutput::PublishOdometry, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        linear_filter->publish_map = std::bind(&LocaliserOutput::PublishMap, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        linear_filter->publish_statistics = std::bind(&LocaliserOutput::PublishStatistics, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
        break;

     case 1://MAP_ON
        ROS_WARN_STREAM("MAP_ON: localiser switched to using map-icp updates");
        gnss->perform_update = nullptr;//std::function<void()>();
        map_icp->perform_update = std::bind(&LocaliserInput::Update, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        break;

     case 3://ABS_ON
        ROS_WARN_STREAM("ABS_ON: localiser switched to using GPS updates");
        gnss->perform_update = std::bind(&LocaliserInput::Update, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
        map_icp->perform_update = nullptr;//std::function<void()>();
        break;

     default:
        ROS_WARN_STREAM("Currently undefined localiser instruction type " << int(req.instructionType));

  }

  res.status = true;

  return true;
}


void
ROSLocaliser::Initialise() {


    /*********************************************************
     *  topics to use
     */


    /*********************************************************
    *  input sources connect to the ROSInput
    */

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

    // Spin and read messages forever
    ros::spin();
    }
    else {
        ROS_INFO_STREAM("[INPUT] Reading data from bagfile" << input_bag);

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

        // point_cloud topics
        bag_input->pointcloud_topics.insert("/velodyne/front/points");

        // bind bag input functions to appropriate message handlers
        bag_input->publish_odom_update = std::bind(&ICPObservation::receive_message, &(*map_icp), std::placeholders::_1);
        bag_input->publish_speed_update = std::bind(&SpeedMeasurement::receive_message, &(*speed), std::placeholders::_1);
        bag_input->publish_fix_update = std::bind(&GNSSObservation::receive_message, &(*gnss), std::placeholders::_1);
        bag_input->publish_imu_update = std::bind(&ImuMeasurement::receive_message, &(*imu), std::placeholders::_1);

        if (run_pipeline) {

            // todo: make this conditional on doing the point cloud feature pipeline
            features_pipeline = std::make_shared<PointCloudFeaturesPipeline>();
            bag_input->publish_pointcloud_update = std::bind(&PointCloudFeaturesPipeline::receive_message,
                                                           &(*features_pipeline), std::placeholders::_1);

            // todo: make this conditional on doing the ICP matching
            icp_pipeline = std::make_shared<ICPMatcherPipeline>();
            features_pipeline->publish_poles_corners = std::bind(&ICPMatcherPipeline::receive_message, &(*icp_pipeline),
                                                               std::placeholders::_1, std::placeholders::_2);

            icp_pipeline->publish_pose = std::bind(&ICPObservation::receive_message, &(*map_icp), std::placeholders::_1);
        }

    }



  /*********************************************************
     *  connect the ROSInput layer to the localiser input layer
     */

    // bind update method
    // initial state is gnss localisation. will switch to map based localisation if behavior tree sees fit
    // map_icp->perform_update is an empty std::function(not initialized)
  //  map_icp->perform_update = std::bind(&LocaliserInput::Update, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    gnss->perform_update = std::bind(&LocaliserInput::Update, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);

    // bind set variables
    imu->update_pitch = std::bind(&LocaliserInput::SetPitch, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    imu->update_yaw_rate = std::bind(&LocaliserInput::SetYawRate, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    speed->update_speed = std::bind(&LocaliserInput::SetSpeed, localiser_input, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // We want the prediction step to be run whenever the IMU gives a msg (100Hz)
    imu->perform_prediction = std::bind(&LocaliserInput::Predict, localiser_input, std::placeholders::_1);




    /*********************************************************
     *  connect the localiser input to the localiser layer,
     *  and the localiser layer to the ROSOutput layer
     */

    // run using gtsam or graph optimizer
    if (0) {
      /*
      // bind localisation method inputs
      // gtsam_optimiser = std::make_shared<GtsamOptimiser>();
      // localiser_input->perform_update = std::bind(&GtsamOptimiser::AddAbsolutePosition, gtsam_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
      // localiser_input->perform_prediction = std::bind(&GtsamOptimiser::AddRelativeMotion, gtsam_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      //
      // where to send localisation method outputs
      // gtsam_optimiser->publish_odometry = std::bind(&LocaliserOutput::PublishOdometry, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      */
    }
    else if(false){
      // bind localisation method inputs
      localiser_input->perform_update = std::bind(&GraphOptimiser::AddAbsolutePosition, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
      localiser_input->perform_prediction = std::bind(&GraphOptimiser::AddRelativeMotion, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

      // where to send localisation method outputs
      graph_optimiser->publish_odometry = std::bind(&LocaliserOutput::PublishOdometry, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      graph_optimiser->publish_map = std::bind(&LocaliserOutput::PublishMap, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    }
    else if(false){
      // DO WITHOUT SAVING YET TO THE FILE
      // bind localisation method inputs
      localiser_input->perform_update = std::bind(&GraphOptimiser::AddAbsolutePosition, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
      localiser_input->perform_prediction = std::bind(&GraphOptimiser::AddRelativeMotion, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

      graph_optimiser->run_optimiser_each_observation = false;
    }
    else {

      localiser_input->perform_update = std::bind(&LinearFilter::AddAbsolutePosition, linear_filter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
      localiser_input->perform_prediction = std::bind(&LinearFilter::AddRelativeMotion, linear_filter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

      // where to send localisation method outputs
      linear_filter->publish_odometry = std::bind(&LocaliserOutput::PublishOdometry, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
      linear_filter->publish_map = std::bind(&LocaliserOutput::PublishMap, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
      linear_filter->publish_statistics = std::bind(&LocaliserOutput::PublishStatistics, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
    }




    /*********************************************************
     *  connect the ROSOutput layer to the output destinations
     */


    if (output_bag.empty()) {
  //      if (true) {

      ROS_INFO_STREAM("[OUTPUT] Publishing output");
      //localiser_output->publish_odom.push_back(std::function<void(nav_msgs::Odometry&, std::string)>);
      //localiser_output->publish_odom.back() = std::bind(&Publisher::publish_odom, publisher, std::placeholders::_1, std::placeholders::_2);
      localiser_output->publish_odom.push_back(std::bind(&Publisher::publish_odom, publisher, std::placeholders::_1, std::placeholders::_2));
      localiser_output->publish_fix.push_back(std::bind(&Publisher::publish_fix, publisher, std::placeholders::_1, std::placeholders::_2));
      localiser_output->publish_tf.push_back(std::bind(&Publisher::publish_tf, publisher, std::placeholders::_1, std::placeholders::_2));
      localiser_output->publish_stats.push_back(std::bind(&Publisher::publish_stats, publisher, std::placeholders::_1, std::placeholders::_2));
    }
    else {
      ROS_INFO_STREAM("[OUTPUT] Writing to bagfile " << output_bag);
      bag_output->Initialise(output_bag);
      //localiser_output->publish_odom = std::bind(&BagOutput::publish_odom, bag_output, std::placeholders::_1, std::placeholders::_2);

      // publish the odom even though writing to the file - necessary if doing ICP matching
      // todo: make this conditional on doing the ICP matching
      if (run_pipeline) {
        localiser_output->publish_odom.push_back(std::bind(&Publisher::publish_odom, publisher, std::placeholders::_1, std::placeholders::_2));
        localiser_output->publish_tf.push_back(std::bind(&Publisher::publish_tf, publisher, std::placeholders::_1, std::placeholders::_2));
      }

      localiser_output->publish_odom.push_back(std::bind(&BagOutput::publish_odom, bag_output, std::placeholders::_1, std::placeholders::_2));
      localiser_output->publish_fix.push_back(std::bind(&BagOutput::publish_fix, bag_output, std::placeholders::_1, std::placeholders::_2));
      localiser_output->publish_tf.push_back(std::bind(&BagOutput::publish_tf, bag_output, std::placeholders::_1, std::placeholders::_2));
  //    localiser_output->publish_stats.push_back(std::bind(&BagOutput::publish_stats, bag_output, std::placeholders::_1, std::placeholders::_2));
      localiser_output->publish_stats.push_back(std::bind(&Publisher::publish_stats, publisher, std::placeholders::_1, std::placeholders::_2));

    }



    /*********************************************************
     *  start processing
     */

    bag_input->ReadBag(input_bag);


}

// todo: publish pointcloud from pipeline ? allow to make lidar_png. Work out how to use this with partial scans
//  (attach to graph ?, make separate subscans and put in time order with the tf tree buffer?)

// todo: make a launch for all other things required for pointcloudfeatures pipeline

// todo: publish transforms
