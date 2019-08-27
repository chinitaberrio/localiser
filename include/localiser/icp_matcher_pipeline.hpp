//
// Created by stew on 27/08/19.
//

#ifndef LOCALISER_ICP_MATCHER_PIPELINE_HPP
#define LOCALISER_ICP_MATCHER_PIPELINE_HPP

#include "run_pipeline.hpp"

#include "point_xyzir.h"
#include "point_xyzirc.h"

class ICPMatcherPipeline : public RunPipeline {

public:
  ICPMatcherPipeline();
  ~ICPMatcherPipeline() {}

  void receive_message(const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud);

private:
  PipelineInput<pcl::PointCloud<pcl::PointXYZIR>> input_pointcloud;
  PipelineOutput<pcl::PointCloud<pcl::PointXYZIRC>> output_poles, output_corners;

};

#endif //LOCALISER_POINT_CLOUD_FEATURES_PIPELINE_HPP
