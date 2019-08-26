#include "localiser.hpp"

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
}

void
Localiser::SetSpeed(double speed, double variance, ros::Time stamp) {
  measured_speed = speed;
}

void
Localiser::SetYawRate(double yaw_rate, double variance, ros::Time stamp) {
  measured_yaw_rate = yaw_rate;
}

void
Localiser::Predict(ros::Time stamp) {

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

  //  if (measured_speed > 3 && measured_yaw_rate < 0.01 && perform_update) {
  //  if (measured_speed > 2. && perform_update) {
  if (perform_update) {
    perform_update(observation, covariance, stamp);
  }
}



