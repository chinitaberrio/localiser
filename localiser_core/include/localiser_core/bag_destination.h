#ifndef bag_output_h
#define bag_output_h

// include messages to write to bag file
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <dataset_tools/LocaliserStats.h>

#include <tf/transform_datatypes.h>

#include "destination_interface.h"

namespace rosbag {
  class Bag;
}

/*!
 * \brief Class to publish the ros messages
 *
 */

class BagDestination : public DestinationInterface{
public:
  BagDestination(std::string bag_file_name);
  ~BagDestination();

  void write_stats(Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
                                Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time &stamp, std::string &source);

  void write_odom_SE2_msg(std::string &frame_id, std::string &topic_name, Eigen::Vector3d &SE2_estimate,
                                     Eigen::Matrix3d &covariance, ros::Time &stamp);

  void write_map_odom_tf_msg(Eigen::Vector3d &map_SE2_estimate, ros::Time &stamp);

  void write_odom_tf_msg(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp);

  std::shared_ptr<rosbag::Bag> bag;

};




#endif
