﻿#include "bag_destination.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>



#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>





BagDestination::BagDestination(std::string bag_file_name) {

  bag = std::make_shared<rosbag::Bag>();
  // open bag with write mode
  // so that a bag is created/truncated
  bag->open(bag_file_name, rosbag::bagmode::Write);
  // close bag and open again with append operations
  bag->close();
  bag->open(bag_file_name, rosbag::bagmode::Append);

}

BagDestination::~BagDestination() {
    if (bag->isOpen()){
        bag->close();
    }
}


void BagDestination::write_stats(std::string &topic_name, Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
                              Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time &stamp, std::string &source){

  if (bag->isOpen()){
    auto msg = receive_stats2msg(observation, innovation,
                                 covariance, confidence, stamp, source);

    bag->write(topic_name, stamp, msg);
  }
}


void BagDestination::write_odom_SE2_msg(std::string &frame_id, std::string &topic_name, Eigen::Vector3d &SE2_estimate,
                                   Eigen::Matrix3d &covariance, ros::Time &stamp) {

    if (bag->isOpen()){

      auto msg = receive_odom2msg(frame_id, topic_name, SE2_estimate, covariance, stamp);

      bag->write(topic_name, msg.header.stamp, msg);
    }
}


void BagDestination::write_map_odom_tf_msg(Eigen::Vector3d &map_SE2_estimate, ros::Time &stamp) {

  auto msg = receive_map_tf2msg(map_SE2_estimate, stamp);

  if (bag->isOpen()) {
    tf2_msgs::TFMessage transform_msg;
    transform_msg.transforms.push_back(msg);
    bag->write("/tf", stamp, transform_msg);
  }
}

void BagDestination::write_odom_tf_msg(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp) {

  auto msg = receive_odom_tf2msg(odom_SE2_estimate, stamp);

  if (bag->isOpen()) {
    tf2_msgs::TFMessage transform_msg;
    transform_msg.transforms.push_back(msg);
    bag->write("/tf", stamp - ros::Duration(0.00001), transform_msg);
  }
}


