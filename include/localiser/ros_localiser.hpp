#ifndef ros_localiser_h
#define ros_localiser_h

#include <ros/ros.h>

#include "graph_optimiser.hpp"
#include "gtsam_optimiser.hpp"
#include "linear_filter.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <vector>

#include "bag_input.hpp"
#include "bag_output.hpp"
#include "publisher.hpp"

#include "localiser_input.hpp"
#include "localiser_output.hpp"

#include "point_cloud_features_pipeline.hpp"
#include "icp_matcher_pipeline.hpp"

/*!
 * \brief The class that manages the localisation process
 *
 */

class ROSLocaliser {
public:
  ROSLocaliser() {}

  void Initialise();

  std::shared_ptr<BagInput> bag_input;
  std::shared_ptr<BagOutput> bag_output;
  std::shared_ptr<Publisher> publisher;

  std::shared_ptr<LocaliserInput> localiser_input;
  std::shared_ptr<LocaliserOutput> localiser_output;

  std::vector<ros::Subscriber> subscribers;

  std::shared_ptr<PointCloudFeaturesPipeline> features_pipeline;
  std::shared_ptr<ICPMatcherPipeline> icp_pipeline;

  std::shared_ptr<ImuMeasurement> imu;
  std::shared_ptr<SpeedMeasurement> speed;

  std::shared_ptr<ICPObservation> map_icp;
  std::shared_ptr<GNSSObservation> gnss;

  std::shared_ptr<GraphOptimiser> graph_optimiser;
  std::shared_ptr<GtsamOptimiser> gtsam_optimiser;
  std::shared_ptr<LinearFilter> linear_filter;


};




#endif
