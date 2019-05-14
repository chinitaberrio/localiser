#ifndef localiasation_method_h
#define localiasation_method_h

#include <cmath>
#include <string>

#include <mrpt/poses/CPose2D.h>
#include <mrpt_bridge/mrpt_bridge.h>

#include <gps_common/conversions.h>


#include <Eigen/Core>
#include <Eigen/StdVector>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <rosbag/bag.h>


//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


class LocalisationMethod {

public:
  LocalisationMethod();

  //! Callback to publish odometry information
  std::function<void(Eigen::Vector3d&, Eigen::Vector3d&, ros::Time)> publish_odometry;

  //! Callback to publish map information
  std::function<void(Eigen::Vector3d&, Eigen::Vector3d&, ros::Time)> publish_map;

  // Current estimate of the robot state (map frame, odom frame)
  Eigen::Vector3d map_state;
  Eigen::Vector3d odom_state;

  ros::Time previous_prediction_stamp;
  ros::Time previous_observation_stamp;
};


#endif
