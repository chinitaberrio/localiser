#include "publisher.hpp"



#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>



void Publisher::write_odom_SE2_msg(std::string &frame_id, std::string &topic_name, Eigen::Vector3d &SE2_estimate,
                             Eigen::Matrix3d &covariance, ros::Time &stamp) {



    ros::NodeHandle n;
    pub = n.advertise<nav_msgs::Odometry>(topic_name, 100);

    auto msg = receive_odom2msg(frame_id, topic_name, SE2_estimate, covariance, stamp);

    pub.publish(msg);



}


void Publisher::write_stats(Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
                              Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time &stamp, std::string &source) {


    ros::NodeHandle n;
    pub = n.advertise<dataset_tools::LocaliserStats>("Statistics", 100);

    auto msg = receive_stats2msg(observation, innovation,
                                 covariance, confidence, stamp, source);

    pub.publish(msg);



}


void
Publisher::write_map_odom_tf_msg(Eigen::Vector3d &map_SE2_estimate, ros::Time &stamp) {

  auto msg = receive_map_tf2msg(map_SE2_estimate, stamp);

  transform_broadcaster.sendTransform(msg);

}


void
Publisher::write_odom_tf_msg(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp) {

  auto msg = receive_odom_tf2msg(odom_SE2_estimate, stamp);

  transform_broadcaster.sendTransform(msg);

}
