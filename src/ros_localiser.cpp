#include "ros_localiser.hpp"

#include "graph_optimiser.hpp"
#include "gtsam_optimiser.hpp"

#include <rosbag/view.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>


#include <rosbag/bag.h>
#include <rosbag/view.h>


int main(int argc, char** argv) {

  ros::init(argc, argv, "Localiser");

  ROSLocaliser ros_localiser;
  ros_localiser.Initialise();
//  ros::spin();

  return 0;
}


void
ROSLocaliser::PublishMap(Eigen::Vector3d &map_estimate, Eigen::Vector3d &covariance, ros::Time stamp) {
//    ROS_INFO_STREAM("PUBLISH MAP " << map_estimate);


    nav_msgs::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = map_estimate[0];
    msg.pose.pose.position.y = map_estimate[1];

    //msg.pose.orientation;
    map_publisher.publish(msg);



    double lat, lon;
    gps_common::UTMtoLL(map_estimate[1], map_estimate[0], "56H", lat, lon);

    sensor_msgs::NavSatFix fix_msg;
    fix_msg.header.frame_id = "map";
    fix_msg.header.stamp = stamp;
    fix_msg.latitude = lat;
    fix_msg.longitude = lon;

    fix_publisher.publish(fix_msg);
}


void
ROSLocaliser::Initialise() {
  ros::NodeHandle n;

  map_publisher = n.advertise<nav_msgs::Odometry>("map/odometry", 100);
  fix_publisher = n.advertise<sensor_msgs::NavSatFix>("map/fix", 100);
  observe_publisher = n.advertise<sensor_msgs::NavSatFix>("map/raw_fix", 100);

  localiser = std::make_shared<Localiser>();

  // bind inputs
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

  // localisation method
  graph_optimiser = std::make_shared<GraphOptimiser>();

  gtsam_optimiser = std::make_shared<GtsamOptimiser>();


  /*
  // bind localisation method inputs
  localiser->perform_update = std::bind(&GtsamOptimiser::AddAbsolutePosition, gtsam_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  localiser->perform_prediction = std::bind(&GtsamOptimiser::AddRelativeMotion, gtsam_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  // where to send localisation method outputs
  gtsam_optimiser->publish_odometry = std::bind(&ROSLocaliser::PublishOdometry, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
*/
  // bind localisation method inputs
  localiser->perform_update = std::bind(&GraphOptimiser::AddAbsolutePosition, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  localiser->perform_prediction = std::bind(&GraphOptimiser::AddRelativeMotion, graph_optimiser, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  // where to send localisation method outputs
  graph_optimiser->publish_odometry = std::bind(&ROSLocaliser::PublishOdometry, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  graph_optimiser->publish_map = std::bind(&ROSLocaliser::PublishMap, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);



rosbag::Bag bag;
//bag.open("/home/stew/data/2019-02-22_Dataset_year/2019-02-22-16-16-58_Dataset_year.bag");  // BagMode is Read by default
bag.open("/home/stew/data/callan-park/2019-04-15-14-37-06_callan_park_loop.bag");  // BagMode is Read by default
//bag.open("/home/stew/data/2019-03-07_Mainquad_video/2019-03-07-15-34-14_Mainquad_video.bag");  // BagMode is Read by default
for(rosbag::MessageInstance const m: rosbag::View(bag))
{
  //sensor_msgs::NavSatFix
  //sensor_msgs::Imu
  //nav_msgs::Odometry
  auto msg_1 = m.instantiate<sensor_msgs::Imu>();
  if (msg_1 != NULL && m.getTopic() == "/vn100/imu")
    imu->receive_message(msg_1);

  auto msg_2 = m.instantiate<sensor_msgs::NavSatFix>();
  if (msg_2 && m.getTopic() == "/ublox_gps/fix") {
    gnss->receive_message(msg_2);
    observe_publisher.publish(msg_2);
  }

  auto msg_3 = m.instantiate<nav_msgs::Odometry>();
  if (msg_3 && m.getTopic() == "/zio/odometry/rear")
    speed->receive_message(msg_3);
  else if (msg_3 && m.getTopic() == "/vn100/odometry")
    speed->receive_message(msg_3);
  //else if (msg_3 && m.getTopic() == "/localisation/gnss/utm")
  //  map_icp->receive_message(msg_3);

  if (!ros::ok())
    break;
}

bag.close();

/*
  // subscribe
  subscribers.push_back(n.subscribe("/vn100/imu", 1000, &ImuMeasurement::receive_message, &(*imu)));
  subscribers.push_back(n.subscribe("/ublox_gps/fix", 1000, &GNSSObservation::receive_message, &(*gnss)));
  subscribers.push_back(n.subscribe("/zio/odometry/rear", 1000, &SpeedMeasurement::receive_message, &(*speed)));
  subscribers.push_back(n.subscribe("/velodyne/front/fix", 1000, &ICPObservation::receive_message, &(*map_icp)));
*/

  ROS_INFO_STREAM("Initialised Localiser");
}
