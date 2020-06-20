#include "bitmap_output.h"


#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>





BitmapOutput::BitmapOutput() {

}

BitmapOutput::~BitmapOutput() {
}


void BitmapOutput::publish_odom(nav_msgs::Odometry &msg, std::string topic_name) {
  if (bag->isOpen())
    bag->write(topic_name, msg.header.stamp, msg);
}


void BitmapOutput::publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name) {
  if (bag->isOpen())
    bag->write(topic_name, msg.header.stamp, msg);
}



void BagOutput::Initialise(std::string bag_file) {
  bag->open(bag_file, rosbag::bagmode::Write);
}
