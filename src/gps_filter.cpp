#include "gps_filter.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <vector>


int main(int argc, char** argv) {

  ros::init(argc, argv, "gps_filter");
  ros::Time::init();

  GpsFilter gps_filter;
  gps_filter.connect_layers();

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




  // input method: from subscriber or rosbag play
  ros::param::param<std::string>("~input_bag", input_bag_name, "");
  if (input_bag_name.empty()) {
      ROS_ERROR("input bag name required");
  }

  // output method: publish output or write to bag
  ros::param::param<std::string>("~output_bag", output_bag_name, "");
  if (!output_bag_name.empty()) {
    bag_destination = std::make_shared<BagDestination>(output_bag_name);
  }else{
      ROS_ERROR("output bag name required");
  }

}


void
GpsFilter::connect_layers() {


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

      ekf->signal_statistics.push_back(std::bind(&BagDestination::write_stats, bag_destination, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
    }



    /*********************************************************
     *  start processing
     */


        ROS_INFO_STREAM("[INPUT] Reading data from bagfile" << input_bag_name);
        bag_source->ReadBag(input_bag_name);


}

