#ifndef predictor_h
#define predictor_h

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <ros/ros.h>

//#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


/*!
 * \brief The base class for predictive inputs
 * Class type that gives an informed prediction about the future
 * relative location of the robot (given some relative sensor information)
 */

class Predictor {
public:
  Predictor();

  //! Perform the prediction
  std::function<void(ros::Time)> perform_prediction;

  //! Update the yaw rate (and variance) of the robot in radians per second
  std::function<void(double, double, ros::Time)> update_yaw_rate;

  //! Update the speed (and variance) of the robot in meters per second
  std::function<void(double, double, ros::Time)> update_speed;

  //! Update the absolute pitch (and variance) angle of the robot (in radians)
  std::function<void(double, double, ros::Time)> update_pitch;

};


class ImuMeasurement : public Predictor {
public:
  ImuMeasurement() : Predictor() {};

  void receive_message(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion orientation_tf;
    tf2::convert(msg->orientation, orientation_tf);

    if (update_pitch) {
      tf2::Matrix3x3 mat(orientation_tf);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);
      // covariance [4] is y-orientation
      update_pitch(pitch,  msg->orientation_covariance[4], msg->header.stamp);
    }

    if (update_yaw_rate) {
      // covariance [8] is z-angular-velocity
      update_yaw_rate(msg->angular_velocity.z,  msg->orientation_covariance[8], msg->header.stamp);
    }

    if (perform_prediction) {
      perform_prediction(msg->header.stamp);
    }
  }
};


class SpeedMeasurement : public Predictor {
public:
  SpeedMeasurement() : Predictor() {};

  void receive_message(const nav_msgs::Odometry::ConstPtr& msg) {
    if (update_speed) {
      // covariance [0] is x-twist-linear
      update_speed(msg->twist.twist.linear.x, msg->twist.covariance[0], msg->header.stamp);
    }
  }
};



#endif
