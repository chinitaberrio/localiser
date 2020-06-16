#ifndef SOURCEINTERFACE_H
#define SOURCEINTERFACE_H
#include <ros/ros.h>

#include <h264_bag_playback/h264_bag_playback.hpp>
#include <dataset_tools/behavior_tree_pipeline.hpp>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

class SourceInterface
{
public:
    SourceInterface();


    void receive_odom_SE2_msg(const nav_msgs::Odometry::ConstPtr& msg);
    std::function<void(double, double, double)> signal_SE2;


    void receive_fix_msg(const sensor_msgs::NavSatFix::ConstPtr& msg);
    std::function<void(double, double)> signal_lat_lon;


    // If we want to turn off map matching, we don't want the features and ICP pipelines to run and use CPU resource
    // so when we need to expose the binding options at this interface between receive_pointcloud_msg and
    // the pipelines that are implemented in sensor model layer
    // this means we can't isolate ros in the source_interface layer
    void receive_pointcloud_msg(const sensor_msgs::NavSatFix::ConstPtr& msg);
    std::function<void(const sensor_msgs::NavSatFix::ConstPtr&)> signal_pointcloud_msg;


    void receive_IMU_msg(const sensor_msgs::Imu::ConstPtr& msg);
    std::function<void(ros::Time)> perform_prediction;
    std::function<void(double, double, ros::Time)> signal_yaw_rate;
    std::function<void(double, double, ros::Time)> signal_pitch;


    void receive_odom_speed_Msg(const nav_msgs::Odometry::ConstPtr& msg);
    std::function<void(double, double, ros::Time)> signal_speed;


};

#endif // SOURCEINTERFACE_H
