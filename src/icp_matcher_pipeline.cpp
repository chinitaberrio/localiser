#include "icp_matcher_pipeline.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


ICPMatcherPipeline::ICPMatcherPipeline() {

  input_pointcloud.Initialise(std::string("/velodyne/front/points"));

  output_poles.Initialise("/velodyne/front/pole_stacker/average", pipes_out);
  output_corners.Initialise("/velodyne/front/corner_stacker/average", pipes_out);
}


void
ICPMatcherPipeline::receive_message(const sensor_msgs::PointCloud2::ConstPtr& input_pc_sensor_msg) {

  ROS_INFO_STREAM("received pointcloud in pipeline " << input_pc_sensor_msg->fields.size());

  // convert sensor_msg PointCloud2 into pcl message
  pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud_pcl(new pcl::PointCloud<pcl::PointXYZIR>);
  pcl::fromROSMsg(*input_pc_sensor_msg, *input_pointcloud_pcl);

  ResetMessageFlags();

  input_pointcloud.PublishMessage(input_pointcloud_pcl);

  if (WaitForMessages()) {
    ROS_INFO_STREAM("received response " << output_poles.last_message->points.size() << ", "
                                         << output_corners.last_message->points.size());
  }
  else {
    ROS_INFO_STREAM("No messages received");
  }
}