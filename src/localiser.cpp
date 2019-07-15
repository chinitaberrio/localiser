#include "localiser.hpp"

/*
#include <rosbag/view.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
*/

#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>



Localiser::Localiser() :
  measured_pitch(0.),
  measured_speed(0.)
{}


void
Localiser::SetPitch(double pitch, double variance, ros::Time stamp) {
  measured_pitch = pitch;
//    ROS_INFO_STREAM("measured pitch " << pitch);
}

void
Localiser::SetSpeed(double speed, double variance, ros::Time stamp) {
  measured_speed = speed;
//    ROS_INFO_STREAM("measured speed " << speed);
}

void
Localiser::SetYawRate(double yaw_rate, double variance, ros::Time stamp) {
  measured_yaw_rate = yaw_rate;
//    ROS_INFO_STREAM("measured yaw rate " << yaw_rate);
}

void
Localiser::Predict(ros::Time stamp) {
  //ROS_INFO_STREAM("Trigger a prediction step");

  //! The 2d component (horizontal) of the robot speed.
  double speed_horizontal = measured_speed * cos(measured_pitch);

  // Estimate the rate of movement (speed and angular velocity)

  if (perform_prediction) {
    Eigen::Vector2d robot_motion(speed_horizontal, measured_yaw_rate);
    Eigen::Vector2d robot_motion_covariance(measured_speed_variance, measured_yaw_rate_variance);

    perform_prediction(robot_motion, robot_motion_covariance, stamp);
  }
}


void
Localiser::Update(Eigen::Vector3d &observation, Eigen::Vector3d &covariance, ros::Time stamp) {
//  ROS_INFO_STREAM("Incorporate an observation " << measured_speed);

//  if (measured_speed > 3 && measured_yaw_rate < 0.01 && perform_update) {
  if (measured_speed > 2. && perform_update) {
    perform_update(observation, covariance, stamp);
  }
}



