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
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

class SourceInterface
{
public:
    SourceInterface();

    // callback functions(slots) and send signal function pointers(signals) for various rosmsg type

    // IMU
    void receive_IMU_msg(const sensor_msgs::Imu::ConstPtr& msg);
    // We want the prediction calculation to be run whenever the IMU gives a msg (100Hz)
    std::function<void(ros::Time)> signal_calculation;
    std::function<void(double, double, ros::Time)> signal_yaw_rate;
    std::function<void(double, double, ros::Time)> signal_pitch;

    // speed odometer
    void receive_odom_speed_msg(const nav_msgs::Odometry::ConstPtr& msg);
    std::function<void(double, double, ros::Time)> signal_speed;

    // ICP
    void receive_odom_SE2_msg(const nav_msgs::Odometry::ConstPtr& msg);
    std::function<void(double, double, double, ros::Time)> signal_SE2;

    // GPS fix
    void receive_fix_msg(const sensor_msgs::NavSatFix::ConstPtr& msg);
    std::function<void(double, double, int, double, ros::Time)> signal_lat_lon;

    // reset msgs
    void receive_reset_msg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    std::function<void(Eigen::Vector3d& observation, ros::Time& stamp)> signal_reset;


    // point clouds
    // If we want to turn off map matching, we don't want the features and ICP pipelines to run and use CPU resource
    // so when we need to expose the binding options at this interface between receive_pointcloud_msg and
    // the pipelines that are implemented in sensor model layer
    // this means we can't isolate ros in the source_interface layer
    void receive_pointcloud_msg(const sensor_msgs::PointCloud2::ConstPtr& msg);
    std::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> signal_pointcloud_msg;

    double datum_x;
    double datum_y;
    tf::TransformListener transform_listener;


};

#endif // SOURCEINTERFACE_H
