#ifndef localiser_output_h
#define localiser_output_h

#include <ros/ros.h>

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TransformStamped.h>

#include <gps_common/conversions.h>


/*!
 * \brief The class that manages the localisation process
 *
 */

class LocaliserOutput {
public:
  LocaliserOutput();

  tf::TransformBroadcaster transform_broadcaster;
  tf::TransformListener transform_listener;
  float datum_x, datum_y;

  //! Perform the prediction
  void PublishOdometry(Eigen::Vector3d &odometry, Eigen::Matrix3d &covariance, ros::Time stamp);

  // functions to bind to that will provide the ros output messages
  std::function<void(nav_msgs::Odometry&, std::string)> publish_odom;
  std::function<void(sensor_msgs::NavSatFix&, std::string)> publish_fix;

  void PublishMap(Eigen::Vector3d &map_estimate, Eigen::Matrix3d &covariance, Eigen::Vector3d &odom_delta, ros::Time stamp);

  tf2_ros::Buffer transform_buffer;

private:

};



#endif
