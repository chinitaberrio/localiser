#include "run_pipeline.hpp"


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "point_xyzir.h"
#include "point_xyzirc.h"


RunPipeline::RunPipeline() {

}

RunPipeline::~RunPipeline() {
}

/*
void BagOutput::publish_odom(nav_msgs::Odometry &msg, std::string topic_name) {
  if (bag.isOpen())
    bag.write(topic_name, msg.header.stamp, msg);
}


void BagOutput::publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name) {
  if (bag.isOpen())
    bag.write(topic_name, msg.header.stamp, msg);
}
*/

/*
bool
RunPipeline::PublishLatestTransform(std::string parent_frame, std::string child_frame) {

  // find the transform from the buffer

  // publish the transform


  return true;
}
*/


PointCloudFeaturesPipeline::PointCloudFeaturesPipeline() :
    pole_msg_received(false),
    corner_msg_received(false) {

  //std::map<std::string, ros::Subscriber> subscribers;
  //std::map<std::string, ros::Publisher> publishers;

  ros::NodeHandle n;
  subscribers.insert(std::make_pair("/velodyne/front/pole_stacker/average",
      ros::Subscriber(n.subscribe<pcl::PointCloud<pcl::PointXYZIRC>>("/velodyne/front/pole_stacker/average", 1, &PointCloudFeaturesPipeline::PolesCB, this))));

  subscribers.insert(std::make_pair("/velodyne/front/corner_stacker/average",
      ros::Subscriber(n.subscribe<pcl::PointCloud<pcl::PointXYZIRC>>("/velodyne/front/corner_stacker/average", 1, &PointCloudFeaturesPipeline::CornersCB, this))));

  publishers.insert(std::make_pair("/velodyne/front/filtered", ros::Publisher(n.advertise<pcl::PointCloud<pcl::PointXYZIR>>("/velodyne/front/points", 1))));

}



PointCloudFeaturesPipeline::~PointCloudFeaturesPipeline() {

}


void
PointCloudFeaturesPipeline::PolesCB(const pcl::PointCloud<pcl::PointXYZIRC>::ConstPtr& pointcloud) {

  pole_msg_received = true;
}


void
PointCloudFeaturesPipeline::CornersCB(const pcl::PointCloud<pcl::PointXYZIRC>::ConstPtr& pointcloud) {

  corner_msg_received = true;

}



void
PointCloudFeaturesPipeline::receive_message(const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud) {
  ROS_INFO_STREAM("received pointcloud in pipeline " << input_pointcloud->fields.size());


  pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud_pcl(new pcl::PointCloud<pcl::PointXYZIR>);

  pcl::fromROSMsg(*input_pointcloud, *input_pointcloud_pcl);

  ROS_INFO_STREAM("completed conversions");
  pcl::PointCloud<pcl::PointXYZIRC>::Ptr pole_pointcloud(new pcl::PointCloud<pcl::PointXYZIRC>), corner_pointcloud(new pcl::PointCloud<pcl::PointXYZIRC>);

  pcl::PointCloud<pcl::PointXYZIR>::ConstPtr const_input_pointcloud_pcl = input_pointcloud_pcl;
  FeatureExtractor(const_input_pointcloud_pcl, pole_pointcloud, corner_pointcloud);
}




void
PointCloudFeaturesPipeline::FeatureExtractor(pcl::PointCloud<pcl::PointXYZIR>::ConstPtr& input_pointcloud,
                      pcl::PointCloud<pcl::PointXYZIRC>::Ptr& pole_pointcloud,
                      pcl::PointCloud<pcl::PointXYZIRC>::Ptr& corner_pointcloud) {


  //sensor_msgs::PointCloud2 input_pointcloud_msg;
////////////////////////  pcl::toROSMsg(input_pointcloud, *input_pointcloud_msg);

  //aligned_pointcloud_msg.header.frame_id = "base_link_horizon";

  // TODO: write the aligned pointcloud to the rosbag
  ///////  out_bag.write("/pointcloud", pointcloud->header.stamp, aligned_pointcloud);

  // add the time of the pose to the map
  //poseid_time_map.insert(std::pair<unsigned int, ros::Time>(optimizer->latest_robot_pose_id, input_pointcloud_msg->header.stamp));

  // store the latest pointcloud time (The pipeline messages should output this same time
  //latest_pointcloud_stamp = input_pointcloud_msg->header.stamp;

  // publish the transform to the pipeline
  //baselink_tf_msg.header.frame_id = std::string("base_link_horizon");
  //tf2_broadcaster.sendTransform(baselink_tf_msg);

  // TODO: is this supposed to be the aligned one ?
  // publish the pointcloud to the pipeline
  //cloud_pub_.publish(aligned_pointcloud_msg);

  ROS_INFO_STREAM("publishing message points with " << input_pointcloud->points.size());
  ROS_INFO_STREAM("to publisher " << publishers["/velodyne/front/filtered"].getTopic());

  //publishers.front().second.publish(input_pointcloud);
  publishers["/velodyne/front/filtered"].publish(input_pointcloud);

  ROS_INFO_STREAM("wait for response");

  // block while waiting for the pipeline to finish
  while (!(corner_msg_received && pole_msg_received) && ros::ok()){
    ros::spinOnce();
  }

  ROS_INFO_STREAM("received response");

  // reset the flags
  corner_msg_received = false;
  pole_msg_received = false;


}
