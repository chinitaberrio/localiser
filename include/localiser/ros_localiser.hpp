#ifndef ros_localiser_h
#define ros_localiser_h

#include <ros/ros.h>

#include "graph_optimiser.hpp"
#include "gtsam_optimiser.hpp"

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>

#include "bag_input.hpp"
#include "bag_output.hpp"
#include "localiser.hpp"
#include "publisher.hpp"

#include "run_pipeline.hpp"

/*!
 * \brief The class that manages the localisation process
 *
 */

class ROSLocaliser {
public:
  ROSLocaliser()  {}

  void Initialise();

  //! Perform the prediction
  void PublishOdometry(Eigen::Vector3d &odometry, Eigen::Vector3d &covariance, ros::Time stamp);

  // functions to bind to that will provide the ros output messages
  std::function<void(nav_msgs::Odometry&, std::string)> publish_odom;
  std::function<void(sensor_msgs::NavSatFix&, std::string)> publish_fix;

  void PublishMap(Eigen::Vector3d &map_estimate, Eigen::Vector3d &covariance, ros::Time stamp);

  std::shared_ptr<BagInput> bag_input;
  std::shared_ptr<BagOutput> bag_output;
  std::shared_ptr<Publisher> publisher;
  std::shared_ptr<Localiser> localiser;

  std::vector<ros::Subscriber> subscribers;

  std::shared_ptr<PointCloudFeaturesPipeline> features_pipeline;

  std::shared_ptr<ImuMeasurement> imu;
  std::shared_ptr<SpeedMeasurement> speed;

  std::shared_ptr<ICPObservation> map_icp;
  std::shared_ptr<GNSSObservation> gnss;

  std::shared_ptr<GraphOptimiser> graph_optimiser;
  std::shared_ptr<GtsamOptimiser> gtsam_optimiser;


};




#endif
