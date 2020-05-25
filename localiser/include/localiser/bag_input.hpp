//
// Created by stew on 25/05/20.
//

#ifndef LOCALISER_BAG_INPUT_HPP
#define LOCALISER_BAG_INPUT_HPP

#include <ros/ros.h>

#include <h264_bag_playback/h264_bag_playback.hpp>

// include messages to write to bag file
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>



/*!
 * \brief Class to publish the ros messages
 *
 */

class BagInput : public dataset_toolkit::h264_bag_playback {
public:
  BagInput();

  void ReadBag(std::string bag_file);

  void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message);
  void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {}
  void CameraInfoPublisher(ros::Publisher &publisher, const sensor_msgs::CameraInfoConstPtr &message) {}



  std::function<void(const nav_msgs::Odometry::ConstPtr&)> publish_odom_update;
  std::function<void(const sensor_msgs::NavSatFix::ConstPtr&)> publish_fix_update;
  std::function<void(const nav_msgs::Odometry::ConstPtr&)> publish_speed_update;
  std::function<void(const sensor_msgs::Imu::ConstPtr&)> publish_imu_update;
  std::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> publish_pointcloud_update;

  std::set<std::string> odom_update_topics;
  std::set<std::string> fix_update_topics;
  std::set<std::string> odom_speed_topics;
  std::set<std::string> pointcloud_topics;
  std::set<std::string> imu_topics;


};



#endif //LOCALISER_BAG_INPUT_HPP
