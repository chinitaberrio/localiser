#ifndef publisher_h
#define publisher_h

#include <map>

#include <ros/ros.h>

// include messages to publish

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_msgs/TFMessage.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <dataset_tools/LocaliserStats.h>
#include "destination_interface.h"

/*!
 * \brief Class to publish the ros messages
 *
 */

class Publisher : public DestinationInterface {
public:
  Publisher();

  void publish_odom(std::string &frame_id, std::string &topic_name, Eigen::Vector3d &SE2_estimate,
                               Eigen::Matrix3d &covariance, ros::Time &stamp);

  void publish_stats(std::string &topic_name, Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
                                Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time &stamp, std::string &source);

  void publish_map_odom_tf(Eigen::Vector3d &map_SE2_estimate, ros::Time &stamp);

  void publish_odom_tf(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp);

//  std::map<std::string, ros::Publisher> publishers;
  tf::TransformBroadcaster transform_broadcaster;
  ros::Publisher pub;


};




#endif
