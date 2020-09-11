#ifndef GPSFILTER_H
#define GPSFILTER_H


#include <ros/ros.h>

#include "localiser_core/linear_filter.h"
#include "localiser_core/graph_optimiser.h"
//#include "localiser_core/gtsam_optimiser.h"

#include "localiser_core/bag_source.h"

#include "localiser_core/observation_model.h"
#include "localiser_core/motion_model.h"

#include "localiser_core/bag_destination.h"
#include "localiser_core/publisher.h"

#include "localiser/reset_localiser.h"
#include <dataset_tools/LocaliserStats.h>



class GpsFilter
{
public:
    GpsFilter();

private:

    void connect_layers_online();
    void connect_layers_offline();


    std::shared_ptr<BagSource> bag_source;

    std::shared_ptr<GNSSObservation> gnss;

    std::shared_ptr<MotionModel> ekf_motion_model;

  //  std::shared_ptr<GraphOptimiser> graph_optimiser;
  //  std::shared_ptr<GtsamOptimiser> gtsam_optimiser;
    std::shared_ptr<PositionHeadingEKF> ekf;

    std::shared_ptr<BagDestination> bag_destination;
    std::shared_ptr<Publisher> publisher;


  //  bool run_pipeline;
    std::string output_bag_name;
    std::string input_bag_name;

};

#endif // GPSFILTER_H
