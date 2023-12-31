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
    Publisher(){}

  void write_odom_SE2_msg(std::string &frame_id, std::string &topic_name, Eigen::Vector3d &SE2_estimate,
                               Eigen::Matrix3d &covariance, ros::Time &stamp);

  void write_stats(std::string &topic_name, Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
                                Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time &stamp, std::string &source);

  void write_map_odom_tf_msg(Eigen::Vector3d &map_SE2_estimate, ros::Time &stamp);

  void write_odom_tf_msg(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp);

  void advertise_topics();

  ros::NodeHandle nh;

  tf::TransformBroadcaster transform_broadcaster;

  std::vector<std::string> odom_SE2_topics;
  std::vector<std::string> stats_topics;
  std::map<std::string, ros::Publisher> publishers;


};


#endif
