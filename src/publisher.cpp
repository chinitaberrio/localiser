#include "publisher.hpp"


#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>



void Publisher::publish_odom(nav_msgs::Odometry &msg, std::string topic_name) {

  if (publishers.find(topic_name) == publishers.end()) {
    ros::NodeHandle n;
    publishers[topic_name] = n.advertise<nav_msgs::Odometry>(topic_name, 100);
 }

  publishers[topic_name].publish(msg);
}


void Publisher::publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name) {

  if (publishers.find(topic_name) == publishers.end()) {
    ros::NodeHandle n;
    publishers[topic_name] = n.advertise<sensor_msgs::NavSatFix>(topic_name, 100);
  }

  publishers[topic_name].publish(msg);
}



void Publisher::Initialise() {
  //ros::NodeHandle n;
  //pub = n.advertise<nav_msgs::Odometry>("/localiser/map/odometry", 100);

  //

  //map_publisher = n.advertise<nav_msgs::Odometry>("map/odometry", 100);
  //fix_publisher = n.advertise<sensor_msgs::NavSatFix>("map/fix", 100);
  //observe_publisher = n.advertise<sensor_msgs::NavSatFix>("map/raw_fix", 100);
}
