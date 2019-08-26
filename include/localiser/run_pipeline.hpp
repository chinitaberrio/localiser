#ifndef run_pipeline_h
#define run_pipeline_h

#include <ros/ros.h>

// include messages to write to bag file
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>


//
//#include <Eigen/Core>
//#include <Eigen/StdVector>
//
//#include <vector>



class PipelineInput {
public:

};


class PipelineOutput {
public:

};

/*!
 * \brief Call an external pipeline (ros node) and wait for the responses
 *
 */

class RunPipeline {
public:
  RunPipeline();
  ~RunPipeline();

  /*
  void publish_odom(nav_msgs::Odometry &msg, std::string topic_name);
  void publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name);
  */

//  std::map<std::string, std::

//  void PublishLatestTransform(std::string parent_frame, std::string child_frame);


  std::map<std::string, ros::Subscriber> subscribers;
  std::map<std::string, ros::Publisher> publishers;


};


#include "point_xyzir.h"
#include "point_xyzirc.h"


class PointCloudFeaturesPipeline : public RunPipeline {

public:
  PointCloudFeaturesPipeline();
  ~PointCloudFeaturesPipeline();

  void receive_message(const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud);

  //std::function<void(nav_msgs::Odometry&, std::string)> pipeline_input;
  void FeatureExtractor(pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& input_pointcloud,
      pcl::PointCloud<pcl::PointXYZIRC>::Ptr& pole_pointcloud,
      pcl::PointCloud<pcl::PointXYZIRC>::Ptr& corner_pointcloud);

private:
  void PolesCB(const pcl::PointCloud<pcl::PointXYZIRC>::ConstPtr& pointcloud);
  void CornersCB(const pcl::PointCloud<pcl::PointXYZIRC>::ConstPtr& pointcloud);

  bool pole_msg_received;
  bool corner_msg_received;

  ros::Publisher pub;
};


#endif
