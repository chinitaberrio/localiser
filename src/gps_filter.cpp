#include "gps_filter.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>


int main(int argc, char** argv) {

  ros::init(argc, argv, "gps_filter");
  ros::Time::init();

  GpsFilter gps_filter;
  return 0;
}






// create the localiser modules
GpsFilter::GpsFilter() :
    bag_source(std::make_shared<BagSource>()),
    ekf_motion_model(std::make_shared<MotionModel>()),
    gnss(std::make_shared<GNSSObservation>()),
//    graph_optimiser(std::make_shared<GraphOptimiser>()),
    ekf(std::make_shared<PositionHeadingEKF>()),
    publisher(std::make_shared<Publisher>())
{


  ros::NodeHandle n;

  // output method: publish output or write to bag
  ros::param::param<std::string>("~output_bag", output_bag_name, "");
  if (!output_bag_name.empty()) {
    ROS_INFO_STREAM("GPS filter is writing results to bag " << output_bag_name);
    bag_destination = std::make_shared<BagDestination>(output_bag_name);
  }else{
    ROS_INFO("No output bag name specified." );
  }

  // input method: from subscriber or rosbag play
  ros::param::param<std::string>("~input_bag", input_bag_name, "");
  if (input_bag_name.empty()) {
      ROS_WARN("GPS online filtering mode.");
      connect_layers_online();
  }else{
      ROS_WARN_STREAM("GPS offline filtering mode." << input_bag_name);
      connect_layers_offline();
  }

}


void
GpsFilter::connect_layers_offline() {


    /*********************************************************
     * connect the source interface layer
     *
    *  bag topics
    *  connect to callback function in bag_source
    */

        // disable the interactions with behavior tree
        ROS_WARN("Gps_filter interation with behavior tree is disabled, for faster processing speed");
        bag_source->behavior_tree = false;

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


    /*********************************************************
     * connect signals of the source interface layer to slots of observation/motion model layer
     *
    */
    {
        // bind bag input functions to appropriate message handlers


        // to observation models
//        bag_source->signal_pointcloud_msg = std::bind(&PointcloudObservation::receive_pointcloud, &(*pointcloud), std::placeholders::_1);
//        bag_source->signal_SE2 = std::bind(&ICPObservation::receive_ICP, &(*map_icp), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 );

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
        gnss->signal_update = std::bind(&PositionHeadingEKF::AddAbsolutePosition, ekf, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 );

        ekf_motion_model->signal_prediction = std::bind(&PositionHeadingEKF::AddRelativeMotion, ekf, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    }


    /*********************************************************
     *  connect the localiser kernel layer to the destination interface layer
     */

    // connect publishers
//    {
//        ROS_INFO("[OUTPUT] publishing to topics ");
//        publisher->odom_SE2_topics.push_back(std::string("odometry"));
//        publisher->odom_SE2_topics.push_back(std::string("utm"));

//        publisher->advertise_topics();

//        ekf->signal_odom_state.push_back(std::bind(&Publisher::write_odom_SE2_msg, publisher, std::string("odom"), std::string("odometry"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//        ekf->signal_odom_state.push_back(std::bind(&Publisher::write_odom_tf_msg, publisher,  std::placeholders::_1, std::placeholders::_3));

//        ekf->signal_map_state.push_back(std::bind(&Publisher::write_odom_SE2_msg, publisher, std::string("utm"), std::string("utm"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//        ekf->signal_map_state.push_back(std::bind(&Publisher::write_map_odom_tf_msg, publisher,  std::placeholders::_1, std::placeholders::_3));

//        ekf->signal_statistics.push_back(std::bind(&Publisher::write_stats, publisher, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
//    }

    // connect bag writers
    if (!output_bag_name.empty()) {
      ROS_INFO_STREAM("[OUTPUT] Writing to bagfile " << output_bag_name);

      ekf->signal_odom_state.push_back(std::bind(&BagDestination::write_odom_SE2_msg, bag_destination, std::string("odom"), std::string("/localiser/odometry"),  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      ekf->signal_odom_state.push_back(std::bind(&BagDestination::write_odom_tf_msg, bag_destination,  std::placeholders::_1, std::placeholders::_3));

      ekf->signal_map_state.push_back(std::bind(&BagDestination::write_odom_SE2_msg, bag_destination, std::string("utm"), std::string("/localiser/utm"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
      ekf->signal_map_state.push_back(std::bind(&BagDestination::write_map_odom_tf_msg, bag_destination,  std::placeholders::_1, std::placeholders::_3));

      ekf->signal_update_stats.push_back(std::bind(&BagDestination::write_stats, bag_destination, std::string("/localiser/update_stats"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
//      ekf->signal_statistics.push_back(std::bind(&BagDestination::write_stats, bag_destination, std::string("/localiser/statistics"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));

    }



    /*********************************************************
     *  start processing
     */


        ROS_INFO_STREAM("[INPUT] Reading data from bagfile" << input_bag_name);
        bag_source->ReadBag(input_bag_name);


}



void
GpsFilter::connect_layers_online() {


    /*********************************************************
     * connect the source interface layer
     *
    *  subscribe to topics
    *  connect to callback function in bag_source
    */

    // disable the interactions with behavior tree(don't publish clock ticks for bhTree)
    ROS_WARN("Gps_filter interation with behavior tree is disabled, for faster processing speed");
    bag_source->behavior_tree = false;

    // subscribe
    ros::NodeHandle n;
    std::vector<ros::Subscriber> subscribers;
    // shared_ptr::get() get the raw pointer stored in shared pointer
    SourceInterface* bag_source_raw_ptr = bag_source.get();
    subscribers.push_back(n.subscribe("/vn100/imu", 100, &BagSource::receive_IMU_msg, bag_source_raw_ptr));
    subscribers.push_back(n.subscribe("/ublox_gps/fix", 10, &BagSource::receive_fix_msg, bag_source_raw_ptr));
    subscribers.push_back(n.subscribe("/zio/odometry/rear", 100, &BagSource::receive_odom_speed_msg, bag_source_raw_ptr));
    subscribers.push_back(n.subscribe("/localiser/icp_matcher/odom_corrected", 1, &BagSource::receive_odom_SE2_msg, bag_source_raw_ptr));
    subscribers.push_back(n.subscribe("/initialpose", 1, &BagSource::receive_reset_msg, bag_source_raw_ptr));



    /*********************************************************
     * connect signals of the source interface layer to slots of observation/motion model layer
     *
    */
    {
        // bind bag input functions to appropriate message handlers


        // to observation models
//        bag_source->signal_pointcloud_msg = std::bind(&PointcloudObservation::receive_pointcloud, &(*pointcloud), std::placeholders::_1);
//        bag_source->signal_SE2 = std::bind(&ICPObservation::receive_ICP, &(*map_icp), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 );

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
        gnss->signal_update = std::bind(&PositionHeadingEKF::AddAbsolutePosition, ekf, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4 );

        ekf_motion_model->signal_prediction = std::bind(&PositionHeadingEKF::AddRelativeMotion, ekf, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

        bag_source->signal_reset = std::bind(&PositionHeadingEKF::reinitialise, ekf, std::placeholders::_1, std::placeholders::_2);

    }


    /*********************************************************
     *  connect the localiser kernel layer to the destination interface layer
     */

    // connect publishers
    {
        ROS_INFO("[OUTPUT] publishing to topics ");
        publisher->odom_SE2_topics.push_back(std::string("odometry"));
        publisher->odom_SE2_topics.push_back(std::string("utm"));
        publisher->stats_topics.push_back(std::string("update_stats"));

        publisher->advertise_topics();

        ekf->signal_odom_state.push_back(std::bind(&Publisher::write_odom_SE2_msg, publisher, std::string("odom"), std::string("odometry"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        ekf->signal_odom_state.push_back(std::bind(&Publisher::write_odom_tf_msg, publisher,  std::placeholders::_1, std::placeholders::_3));

        // publish map odometry msg - necessary if doing ICP matching
        ekf->signal_map_state.push_back(std::bind(&Publisher::write_odom_SE2_msg, publisher, std::string("utm"), std::string("utm"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        ekf->signal_map_state.push_back(std::bind(&Publisher::write_map_odom_tf_msg, publisher,  std::placeholders::_1, std::placeholders::_3));

        ekf->signal_update_stats.push_back(std::bind(&Publisher::write_stats, publisher, std::string("update_stats"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
//        ekf->signal_statistics.push_back(std::bind(&Publisher::write_stats, publisher, std::string("statistics"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
    }

    // connect bag writers
//    if (!output_bag_name.empty()) {
//      ROS_INFO_STREAM("[OUTPUT] Writing to bagfile " << output_bag_name);

//      ekf->signal_odom_state.push_back(std::bind(&BagDestination::write_odom_SE2_msg, bag_destination, std::string("odom"), std::string("/localiser/odometry"),  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//      ekf->signal_odom_state.push_back(std::bind(&BagDestination::write_odom_tf_msg, bag_destination,  std::placeholders::_1, std::placeholders::_3));

//      ekf->signal_map_state.push_back(std::bind(&BagDestination::write_odom_SE2_msg, bag_destination, std::string("utm"), std::string("/localiser/utm"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//      ekf->signal_map_state.push_back(std::bind(&BagDestination::write_map_odom_tf_msg, bag_destination,  std::placeholders::_1, std::placeholders::_3));

//      ekf->signal_update_stats.push_back(std::bind(&BagDestination::write_stats, bag_destination, std::string("/localiser/update_stats"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
////      ekf->signal_statistics.push_back(std::bind(&BagDestination::write_stats, bag_destination, std::string("/localiser/statistics"), std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));

//    }



    /*********************************************************
     *  start processing
     */

    ROS_INFO("[INPUT] Subscribing to topics");
    // Spin and read messages forever
    ros::spin();

//        ROS_INFO_STREAM("[INPUT] Reading data from bagfile" << input_bag_name);
//        bag_source->ReadBag(input_bag_name);


}


