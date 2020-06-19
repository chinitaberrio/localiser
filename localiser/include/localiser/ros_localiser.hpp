#ifndef ros_localiser_h
#define ros_localiser_h

#include <ros/ros.h>

#include "localiser_core/graph_optimiser.hpp"
//#include "localiser_core/gtsam_optimiser.hpp"
#include "localiser_core/linear_filter.hpp"

#include "bag_source.hpp"
#include "dataset_tools/bag_output.hpp"
#include "localiser_core/publisher.hpp"

#include "localiser_core/localiser_input.hpp"
#include "localiser_core/localiser_output.hpp"

#include <dataset_tools/point_cloud_features_pipeline.hpp>
#include <dataset_tools/icp_matcher_pipeline.hpp>

#include "localiser/instruct_localiser.h"

#include <dataset_tools/LocaliserStats.h>


/*!
 * \brief The class that manages the localisation process
 *
 */

class ROSLocaliser {
public:

  ROSLocaliser();

  void Initialise();

  std::shared_ptr<BagSource> bag_source;





  std::shared_ptr<ICPObservation> map_icp;
  std::shared_ptr<GNSSObservation> gnss;
  std::shared_ptr<PointcloudObservation> pointcloud;


  std::shared_ptr<MotionModel> ekf_motion_model;




  std::shared_ptr<GraphOptimiser> graph_optimiser;
//  std::shared_ptr<GtsamOptimiser> gtsam_optimiser;
  std::shared_ptr<PositionHeadingEKF> ekf;


  std::shared_ptr<BagDestination> bag_destination;
  std::shared_ptr<Publisher> publisher;




  ros::ServiceServer service;


  bool InstructionCallback(localiser::instruct_localiser::Request& req,
                           localiser::instruct_localiser::Response& res);


  bool run_pipeline;
  std::string output_bag_name;
  std::string input_bag_name;


};




#endif
