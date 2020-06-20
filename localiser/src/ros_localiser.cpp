//#define BOOST_BIND_NO_PLACEHOLDERS

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
  ros_localiser.connect_layers();

  return 0;
}






// create the localiser modules
ROSLocaliser::ROSLocaliser() :
    bag_source(std::make_shared<BagSource>()),
    ekf_motion_model(std::make_shared<MotionModel>()),
    map_icp(std::make_shared<ICPObservation>()),
    gnss(std::make_shared<GNSSObservation>()),
//    graph_optimiser(std::make_shared<GraphOptimiser>()),
    ekf(std::make_shared<PositionHeadingEKF>()),
    publisher(std::make_shared<Publisher>())
{


  ros::NodeHandle n;
  service = n.advertiseService("instruct_localiser", &ROSLocaliser::InstructionCallback, this);

  //! Read the parameters to determine how the localiser will work
//  run_pipeline = false;
//  ros::param::param<bool>("~run_pipeline", run_pipeline, false);

  // output method: publish output or write to bag
  ros::param::param<std::string>("~output_bag", output_bag_name, "");
  bag_destination = std::make_shared<BagDestination>(output_bag_name);

  pointcloud = std::make_shared<PointcloudObservation>(bag_source); // pointcloud signals bag_souce's receive ICP function

  // input method: from subscriber or rosbag play
  ros::param::param<std::string>("~input_bag", input_bag_name, "");

}


void
ROSLocaliser::connect_layers() {


    /*********************************************************
     * connect the source interface layer
     *
    *  slots: ros topics or bag topics to use
    *  signals: to motion/observation models
    */

    std::vector<ros::Subscriber> subscribers;
    if (input_bag_name.empty()) {
    ROS_INFO_STREAM("[INPUT] Subscribing to topics");
    // subscribe

    ros::NodeHandle n;
//    subscribers.push_back(n.subscribe("/vn100/imu", 1000, &BagSource::receive_IMU_msg, &(*bag_source)));
//    subscribers.push_back(n.subscribe("/ublox_gps/fix", 1000, &BagSource::receive_fix_msg, &(*bag_source)));
//    subscribers.push_back(n.subscribe("/zio/odometry/rear", 1000, &BagSource::receive_odom_speed_msg, &(*bag_source)));
//    subscribers.push_back(n.subscribe("/localisation/icp/icp_matcher/odom_corrected", 1000, &BagSource::receive_odom_SE2_msg, &(*bag_source)));

    //todo: make a list of topics for each type of message
    //todo: in the localiser_input/output set the bind/unbind as required


    }else {
        ROS_INFO_STREAM("[INPUT] Reading data from bagfile" << input_bag_name);

        // odometry update messages (i.e. from ICP)
        // bag_source.odom_SE2_topics.insert("/localisation/gnss/utm")

        // GNSS fix messages
        bag_source->fix_topics.insert("/ublox_gps/fix");
//        bag_source->fix_topics.insert("ibeo/gnss");

        // speed odometry messages
        bag_source->odom_speed_topics.insert("/zio/odometry/rear");
//        bag_source->odom_speed_topics.insert("/vn100/odometry");
//        bag_source->odom_speed_topics.insert("ibeo/odometry");

        // imu topics
        bag_source->imu_topics.insert("/vn100/imu");
//        bag_source->imu_topics.insert("xsens/IMU");

        // point_cloud topics
        bag_source->pointcloud_topics.insert("/velodyne/front/points");

        // bind bag input functions to appropriate message handlers

        // to observation models
        bag_source->signal_pointcloud_msg = std::bind(&PointcloudObservation::receive_pointcloud, &(*pointcloud), std::placeholders::_1);
        bag_source->signal_SE2 = std::bind(&ICPObservation::receive_ICP, &(*map_icp), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 );

        bag_source->signal_lat_lon = std::bind(&GNSSObservation::receive_gps, &(*gnss), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5 );


        // to motion models
        bag_source->signal_speed = std::bind(&MotionModel::receive_speed, &(*ekf_motion_model), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        bag_source->signal_yaw_rate = std::bind(&MotionModel::receive_yaw_rate, &(*ekf_motion_model), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        bag_source->signal_pitch = std::bind(&MotionModel::receive_pitch, &(*ekf_motion_model), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        // We want the prediction calculation to be run whenever the IMU gives a msg (100Hz)
        bag_source->signal_calculation = std::bind(&MotionModel::calculate_pose_increment, &(*ekf_motion_model), std::placeholders::_1);
    }



    /*********************************************************
     *  connect the motion/observation models layer to the localiser kernel layer
     */

    {
        // bind update method
        // initial state is gnss localisation. will switch to map based localisation if behavior tree sees fit
        gnss->signal_update = std::bind(&PositionHeadingEKF::AddAbsolutePosition, ekf, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 );
        map_icp->signal_update = std::bind(&PositionHeadingEKF::AddAbsolutePosition, ekf, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 );

        ekf_motion_model->signal_prediction = std::bind(&PositionHeadingEKF::AddRelativeMotion, ekf, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    }


    /*********************************************************
     *  connect the localiser kernel layer to the destination interface layer
     */

    // connect publishers
    {
        ekf->signal_odom_state.push_back(std::bind(&Publisher::write_odom_SE2_msg, publisher, std::string("odom"), std::string("Odometry"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        ekf->signal_odom_state.push_back(std::bind(&Publisher::write_odom_tf_msg, publisher,  std::placeholders::_1, std::placeholders::_3));

        // publish map odometry msg - necessary if doing ICP matching
        ekf->signal_map_state.push_back(std::bind(&Publisher::write_odom_SE2_msg, publisher, std::string("utm"), std::string("UTM"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        ekf->signal_map_state.push_back(std::bind(&Publisher::write_map_odom_tf_msg, publisher,  std::placeholders::_1, std::placeholders::_3));

        ekf->signal_statistics.push_back(std::bind(&Publisher::write_stats, publisher, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
    }

    // connect bag writers
    if (!output_bag_name.empty()) {
      ROS_INFO_STREAM("[OUTPUT] Writing to bagfile " << output_bag_name);


      ekf->signal_odom_state.push_back(std::bind(&BagDestination::write_odom_SE2_msg, bag_destination, std::string("odom"), std::string("Odometry"),  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      ekf->signal_odom_state.push_back(std::bind(&BagDestination::write_odom_tf_msg, bag_destination,  std::placeholders::_1, std::placeholders::_3));

      ekf->signal_map_state.push_back(std::bind(&BagDestination::write_odom_SE2_msg, bag_destination, std::string("utm"), std::string("UTM"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      ekf->signal_map_state.push_back(std::bind(&BagDestination::write_map_odom_tf_msg, bag_destination,  std::placeholders::_1, std::placeholders::_3));

      ekf->signal_statistics.push_back(std::bind(&BagDestination::write_stats, bag_destination, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
    }



    /*********************************************************
     *  start processing
     */

    if(input_bag_name.empty()){
        // Spin and read messages forever
        ros::spin();
    }else{
        bag_source->ReadBag(input_bag_name);
    }

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
using namespace std::placeholders;
     case 0://RESET
        ROS_WARN_STREAM("RESET: localiser reset, reinitialising using GPS");
        bag_source->signal_pointcloud_msg = nullptr;
        bag_source->signal_lat_lon = std::bind(&GNSSObservation::receive_gps, &(*gnss), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5 );

        // make a new object of localisation method
//        linear_filter = std::make_shared<PositionHeadingEKF>();
//        // connect localisation method inputs
//        localiser_input->perform_update = std::bind(&LinearFilter::AddAbsolutePosition, linear_filter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
//        localiser_input->perform_prediction = std::bind(&LinearFilter::AddRelativeMotion, linear_filter, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
//        // where to send localisation method outputs
//        linear_filter->publish_odometry = std::bind(&LocaliserOutput::PublishOdometry, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
//        linear_filter->publish_map = std::bind(&LocaliserOutput::PublishMap, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
//        linear_filter->publish_statistics = std::bind(&LocaliserOutput::PublishStatistics, localiser_output, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6);
        break;

     case 1://MAP_ON
        ROS_WARN_STREAM("MAP_ON: localiser switched to using map-icp updates");
        bag_source->signal_lat_lon = nullptr;
        bag_source->signal_pointcloud_msg = std::bind(&PointcloudObservation::receive_pointcloud, &(*pointcloud), std::placeholders::_1);
        break;

     case 3://ABS_ON
        ROS_WARN_STREAM("ABS_ON: localiser switched to using GPS updates");
        bag_source->signal_lat_lon = std::bind(&GNSSObservation::receive_gps, &(*gnss), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5 );
        bag_source->signal_pointcloud_msg = nullptr;//std::function<void()>();
        break;

     default:
        ROS_WARN_STREAM("Currently undefined localiser instruction type " << int(req.instructionType));

  }

  res.status = true;

  return true;
}

