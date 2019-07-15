#ifndef ros_localiser_h
#define ros_localiser_h

#include <ros/ros.h>

#include "graph_optimiser.hpp"
#include "gtsam_optimiser.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <rosbag/bag.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>

#include "localiser.hpp"




/*!
 * \brief The class that manages the localisation process
 *
 */

class ROSLocaliser {
public:
  ROSLocaliser()  {}

  void Initialise();

  //! Perform the prediction
  void PublishOdometry(Eigen::Vector3d &odometry, Eigen::Vector3d &covariance, ros::Time stamp) {
//    ROS_INFO_STREAM("PUBLISH odometry");


  }

  void PublishMap(Eigen::Vector3d &map_estimate, Eigen::Vector3d &covariance, ros::Time stamp);

  std::shared_ptr<Localiser> localiser;

  std::vector<ros::Subscriber> subscribers;
  ros::Publisher map_publisher;
  ros::Publisher fix_publisher;
  ros::Publisher observe_publisher;

  std::shared_ptr<ImuMeasurement> imu;
  std::shared_ptr<SpeedMeasurement> speed;

  std::shared_ptr<ICPObservation> map_icp;
  std::shared_ptr<GNSSObservation> gnss;

  std::shared_ptr<GraphOptimiser> graph_optimiser;
  std::shared_ptr<GtsamOptimiser> gtsam_optimiser;


};




#endif
