#include "bag_destination.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>



#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>





BagDestination::BagDestination(std::string bag_file_name) {

  bag = std::make_shared<rosbag::Bag>();
  bag->open(bag_file_name, rosbag::bagmode::Write);

}

BagDestination::~BagDestination() {
  bag->close();
}


void BagDestination::write_stats(Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
                              Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time &stamp, std::string &source){

  if (bag->isOpen()){
    auto msg = receive_stats2msg(observation, innovation,
                                 covariance, confidence, stamp, source);

    bag->write("statistics", stamp, msg);
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
    bag->write("/tf", stamp, msg);
  }
}

void BagDestination::write_odom_tf_msg(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp) {

  auto msg = receive_odom_tf2msg(odom_SE2_estimate, stamp);

  if (bag->isOpen()) {
    bag->write("/tf", stamp, msg);
  }
}


