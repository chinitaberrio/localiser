#include "ros_localiser.hpp"

#include "graph_optimiser.hpp"
#include "gtsam_optimiser.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

/*
#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
*/


#include <Eigen/Core>
#include <Eigen/StdVector>



int main(int argc, char** argv) {

  ros::init(argc, argv, "Localiser");
  ros::Time::init();

  ROSLocaliser ros_localiser;
  ros_localiser.Initialise();

  return 0;
}



void
ROSLocaliser::PublishOdometry(Eigen::Vector3d &odometry, Eigen::Vector3d &covariance, ros::Time stamp) {

  // generate an global odometry pose message
  nav_msgs::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "odom";
  msg.pose.pose.position.x = odometry[0];
  msg.pose.pose.position.y = odometry[1];
  // TODO: incorporate orientation
  //msg.pose.orientation;

  if (publish_odom) {
    publish_odom(msg, "/localiser/odometry");
  }

}



void
ROSLocaliser::PublishMap(Eigen::Vector3d &map_estimate, Eigen::Vector3d &covariance, ros::Time stamp) {

  // generate an global odometry pose message
  nav_msgs::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = map_estimate[0];
  msg.pose.pose.position.y = map_estimate[1];
  // TODO: incorporate orientation
  //msg.pose.orientation;

  if (publish_odom) {
    publish_odom(msg, "/localiser/map");
  }

  // generate an global navsatfix message
  double lat, lon;
  gps_common::UTMtoLL(map_estimate[1], map_estimate[0], "56H", lat, lon);

  sensor_msgs::NavSatFix fix_msg;
  fix_msg.header.frame_id = "map";
  fix_msg.header.stamp = stamp;
  fix_msg.latitude = lat;
  fix_msg.longitude = lon;

  if (publish_fix) {
    publish_fix(fix_msg, "/localiser/fix");
  }
}


void
ROSLocaliser::Initialise() {

  // create the localiser manager
  localiser = std::make_shared<Localiser>();

  // bind localiser inputs
  imu = std::make_shared<ImuMeasurement>();
  speed = std::make_shared<SpeedMeasurement>();
  map_icp = std::make_shared<ICPObservation>();
  gnss = std::make_shared<GNSSObservation>();

  // bind update method
  map_icp->perform_update = std::bind(&Localiser::Update, localiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  gnss->perform_update = std::bind(&Localiser::Update, localiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  // bind set variables
  imu->update_pitch = std::bind(&Localiser::SetPitch, localiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  imu->update_yaw_rate = std::bind(&Localiser::SetYawRate, localiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  speed->update_speed = std::bind(&Localiser::SetSpeed, localiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  // bind prediction trigger
  imu->perform_prediction = std::bind(&Localiser::Predict, localiser, std::placeholders::_1);

  // output method
  // publish output or write to bag
  if (0) {
    publisher = std::make_shared<Publisher>();
    this->publish_odom = std::bind(&Publisher::publish_odom, publisher, std::placeholders::_1, std::placeholders::_2);
    this->publish_fix = std::bind(&Publisher::publish_fix, publisher, std::placeholders::_1, std::placeholders::_2);
  }
  else {
    bag_output = std::make_shared<BagOutput>();
    bag_output->Initialise("test_bag.bag");
    this->publish_odom = std::bind(&BagOutput::publish_odom, bag_output, std::placeholders::_1, std::placeholders::_2);
    this->publish_fix = std::bind(&BagOutput::publish_fix, bag_output, std::placeholders::_1, std::placeholders::_2);
  }

  // localisation method
  // run using gtsam or graph optimizer
  if (0) {
    // bind localisation method inputs
    gtsam_optimiser = std::make_shared<GtsamOptimiser>();
    localiser->perform_update = std::bind(&GtsamOptimiser::AddAbsolutePosition, gtsam_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    localiser->perform_prediction = std::bind(&GtsamOptimiser::AddRelativeMotion, gtsam_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // where to send localisation method outputs
    gtsam_optimiser->publish_odometry = std::bind(&ROSLocaliser::PublishOdometry, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  }
  else {
    // bind localisation method inputs
    graph_optimiser = std::make_shared<GraphOptimiser>();
    localiser->perform_update = std::bind(&GraphOptimiser::AddAbsolutePosition, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    localiser->perform_prediction = std::bind(&GraphOptimiser::AddRelativeMotion, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

    // where to send localisation method outputs
    graph_optimiser->publish_odometry = std::bind(&ROSLocaliser::PublishOdometry, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    graph_optimiser->publish_map = std::bind(&ROSLocaliser::PublishMap, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  }

  // input method
  // from subscriber or rosbag play
  if (0) {
    ROS_INFO_STREAM("Initialised Localiser, subscribing to topics");
    // subscribe
    ros::NodeHandle n;
    subscribers.push_back(n.subscribe("/vn100/imu", 1000, &ImuMeasurement::receive_message, &(*imu)));
    subscribers.push_back(n.subscribe("/ublox_gps/fix", 1000, &GNSSObservation::receive_message, &(*gnss)));
    subscribers.push_back(n.subscribe("/zio/odometry/rear", 1000, &SpeedMeasurement::receive_message, &(*speed)));
    subscribers.push_back(n.subscribe("/velodyne/front/poles/average", 1000, &ICPObservation::receive_message, &(*map_icp)));

    ros::spin();
  }
  else {
    ROS_INFO_STREAM("Initialised Localiser, reading from bag");
    bag_input = std::make_shared<BagInput>();

    features_pipeline = std::make_shared<PointCloudFeaturesPipeline>();

    // odometry update messages (i.e. from ICP)
    // bag_input.odom_update_topics.insert("/localisation/gnss/utm")

    // GNSS fix messages
    bag_input->fix_update_topics.insert("/ublox_gps/fix");

    // speed odometry messages
    bag_input->odom_speed_topics.insert("/zio/odometry/rear");
    bag_input->odom_speed_topics.insert("/vn100/odometry");

    // imu topics
    bag_input->imu_topics.insert("/vn100/imu");

    // imu topics
    bag_input->pointcloud_topics.insert("/velodyne/front/filtered");

    // bind bag input functions to appropriate message handlers
    bag_input->publish_odom_update = std::bind(&ICPObservation::receive_message, &(*map_icp), std::placeholders::_1);
    bag_input->publish_speed_update = std::bind(&SpeedMeasurement::receive_message, &(*speed), std::placeholders::_1);
    bag_input->publish_fix_update = std::bind(&GNSSObservation::receive_message, &(*gnss), std::placeholders::_1);
    bag_input->publish_imu_update = std::bind(&ImuMeasurement::receive_message, &(*imu), std::placeholders::_1);

    bag_input->publish_pointcloud_update = std::bind(&PointCloudFeaturesPipeline::receive_message, &(*features_pipeline), std::placeholders::_1);


    bag_input->ReadBag("/home/stew/data/2019-08-14_callan_park_slow_mapping/2019-08-14-12-09-32_callan_park_slow_mapping.bag");

  }
}
