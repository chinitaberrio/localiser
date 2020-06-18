//
// Created by stew on 25/05/20.
//

#ifndef LOCALISER_BAG_INPUT_HPP
#define LOCALISER_BAG_INPUT_HPP

#include <ros/ros.h>

#include <h264_bag_playback/h264_bag_playback.hpp>
#include <dataset_tools/behavior_tree_pipeline.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>



/*!
 * \brief Class to publish the ros messages
 *
 */

class BagSource : public dataset_toolkit::h264_bag_playback, public SourceInterface {
public:
  BagSource();

  void ReadBag(std::string bag_file);

  void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message);
  void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {}
  void CameraInfoPublisher(ros::Publisher &publisher, const sensor_msgs::CameraInfoConstPtr &message) {}

  int odom_msg_count;
  int odom_msg_reset_count;
  std::shared_ptr<BehaviorTreePipeline> behavior_tree_pipeline;


  std::set<std::string> odom_SE2_topics;
  std::set<std::string> fix_topics;
  std::set<std::string> pointcloud_topics;
  std::set<std::string> imu_topics;
  std::set<std::string> odom_speed_topics;

};



#endif //LOCALISER_BAG_INPUT_HPP
