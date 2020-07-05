#include "source_interface.h"

SourceInterface::SourceInterface() : datum_x(0.), datum_y(0.)
{

}


void SourceInterface::receive_IMU_msg(const sensor_msgs::Imu::ConstPtr& msg) {

  if (signal_pitch) {
    tf2::Quaternion orientation_tf;
    tf2::convert(msg->orientation, orientation_tf);
    tf2::Matrix3x3 mat(orientation_tf);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    // covariance [4] is y-orientation
    signal_pitch(pitch,  msg->orientation_covariance[4], msg->header.stamp);
  }

  if (signal_yaw_rate) {
    // covariance [8] is z-angular-velocity
    // not using angular_velocity_covariance, but using orientation_covariance
    // orientation_covariance has static values on diagonal elements
    signal_yaw_rate(msg->angular_velocity.z,  msg->orientation_covariance[8], msg->header.stamp);
  }

  if (signal_calculation) {
    signal_calculation(msg->header.stamp);
  }
}



void SourceInterface::receive_odom_speed_msg(const nav_msgs::Odometry::ConstPtr& msg) {
  if (signal_speed) {
    // covariance [0] is x-twist-linear
    // covariance has static values on diagonal elements
    signal_speed(msg->twist.twist.linear.x, msg->twist.covariance[0], msg->header.stamp);
  }
}


void SourceInterface::receive_odom_SE2_msg(const nav_msgs::Odometry::ConstPtr& msg) {
    if (signal_SE2){

        if (datum_x == 0. || datum_y == 0.) {

          tf::StampedTransform transform;
          try {
            transform_listener.lookupTransform("utm", "map", ros::Time(0), transform);
            datum_x = transform.getOrigin().x();
            datum_y = transform.getOrigin().y();
          }
          catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
          }

        }

        tf2::Quaternion orientation_tf;
        tf2::convert(msg->pose.pose.orientation, orientation_tf);
        tf2::Matrix3x3 mat(orientation_tf);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        if (std::isnan(msg->pose.pose.position.x) ||
            std::isnan(msg->pose.pose.position.y) ) {
          ROS_INFO_STREAM_THROTTLE(1., "rejecting ICP update due to NAN data");
          signal_SE2(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw, msg->header.stamp);
        }else if(msg->header.frame_id == "map"){
            signal_SE2(msg->pose.pose.position.x+datum_x, msg->pose.pose.position.y+datum_y, yaw, msg->header.stamp);
        }else if(msg->header.frame_id == "utm"){
            signal_SE2(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw, msg->header.stamp);
        }
    }
}


void SourceInterface::receive_fix_msg(const sensor_msgs::NavSatFix::ConstPtr& msg) {

//    ROS_WARN("reached SourceInterface::receive_fix_msg");
    if (signal_lat_lon){
//        ROS_WARN("SourceInterface::receive_fix_msg has signal_lat_lon");

        // lat and lon variance are the same, so passing one of them position_covariance[0] is enough
        // don't use attitude variance
        signal_lat_lon(msg->latitude, msg->longitude, msg->status.status, msg->position_covariance[0], msg->header.stamp);
    }
}


// If we want to turn off map matching, we don't want the features and ICP pipelines to run and use CPU resource
// so when we need to expose the binding options at this interface between receive_pointcloud_msg and
// the pipelines that are implemented in sensor model layer
// this means we can't isolate ros in the source_interface layer
void SourceInterface::receive_pointcloud_msg(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (signal_pointcloud_msg){
        signal_pointcloud_msg(msg);
    }
}


