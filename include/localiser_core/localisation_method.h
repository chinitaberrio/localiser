#ifndef localiasation_method_h
#define localiasation_method_h

#include <cmath>
#include <string>

#include <mrpt/poses/CPose2D.h>

#include <gps_common/conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <eigen3/Eigen/Core>


// TODO: vehicle model and odometry calculated in the parent class (this should be common to each method)

class LocalisationMethod {

public:
  LocalisationMethod() :
      odom_state(mrpt::poses::CPose2D(0.,0.,0.)),
      previous_prediction_stamp(ros::Time(0.)),
      previous_observation_stamp(ros::Time(0.))
  {
      last_source = std::make_shared<std::string>("dead_reckon");

  }

  //! Callback to publish odometry information
  std::list<std::function<void(Eigen::Vector3d&, Eigen::Matrix3d&, ros::Time&)>> signal_odom_state;

  //! Callback to publish map information
  std::list<std::function<void(Eigen::Vector3d&, Eigen::Matrix3d&, ros::Time&)>> signal_map_state;

  //! Callback to publish statistics
  std::list<std::function<void(Eigen::Vector3d&, Eigen::Vector3d&,
                               Eigen::Matrix3d&, Eigen::Vector3d&, ros::Time&, std::string&)>> signal_statistics;

  //! Callback to publish statistics
  std::list<std::function<void(Eigen::Vector3d&, Eigen::Vector3d&,
                               Eigen::Matrix3d&, Eigen::Vector3d&, ros::Time&, std::string&)>> signal_update_stats;

  Eigen::Vector3d odom_state_eigen;
  mrpt::poses::CPose2D odom_state;

  ros::Time previous_prediction_stamp;  /////////////
  ros::Time previous_observation_stamp;  ////////////

  std::shared_ptr<std::string> last_source;   //////////////

};


#endif


