#ifndef DESTINATION_INTERFACE_H
#define DESTINATION_INTERFACE_H

// include messages to write to bag file
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <dataset_tools/LocaliserStats.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

class DestinationInterface
{
public:
    DestinationInterface();

    dataset_tools::LocaliserStats
    receive_stats2msg(Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
                Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time &stamp, std::string &source);

    nav_msgs::Odometry
    receive_odom2msg(std::string &frame_id, std::string &topic_name, Eigen::Vector3d &SE2_estimate,
                                       Eigen::Matrix3d &covariance, ros::Time &stamp);

    geometry_msgs::TransformStamped
    receive_map_tf2msg(Eigen::Vector3d &map_SE2_estimate, ros::Time &stamp);

    geometry_msgs::TransformStamped
    receive_odom_tf2msg(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp);


    virtual void write_stats(std::string &topic_name, Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
                             Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time &stamp, std::string &source) = 0;

    virtual void write_odom_SE2_msg(std::string &frame_id, std::string &topic_name, Eigen::Vector3d &SE2_estimate,
                                  Eigen::Matrix3d &covariance, ros::Time &stamp) = 0;

    virtual void write_map_odom_tf_msg(Eigen::Vector3d &map_SE2_estimate, ros::Time &stamp) = 0;

    virtual void write_odom_tf_msg(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp) = 0;


    tf::TransformListener transform_listener;

    double datum_x = 0.;
    double datum_y = 0.;

    Eigen::Vector3d odom_state;
    ros::Time odom_time;
};

#endif // DESTINATION_INTERFACE_H
