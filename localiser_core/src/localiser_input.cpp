#include "localiser_input.hpp"

#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>


LocaliserInput::LocaliserInput() :
  measured_pitch(0.),
  measured_speed(0.)
{}


//void
//LocaliserInput::SetPitch(double pitch, double variance, ros::Time stamp) {
//  measured_pitch = pitch;
//  // todo: add roll, output horizon tf to bag
//}

//void
//LocaliserInput::SetSpeed(double speed, double variance, ros::Time stamp) {
//  measured_speed = speed;
//}

//void
//LocaliserInput::SetYawRate(double yaw_rate, double variance, ros::Time stamp) {
//  measured_yaw_rate = yaw_rate;
//}

void
LocaliserInput::Predict(ros::Time stamp) {


  // Estimate the rate of movement (speed and angular velocity)
  if (perform_prediction) {
      //! The 2d component (horizontal) of the robot speed.
      double speed_horizontal = measured_speed * cos(measured_pitch);

    Eigen::Vector2d robot_motion;
    robot_motion << speed_horizontal, measured_yaw_rate;

    Eigen::Matrix2d robot_motion_covariance;
    robot_motion_covariance << measured_speed_variance, 0.,
        0., measured_yaw_rate_variance;

    perform_prediction(robot_motion, robot_motion_covariance, stamp);
  }
}


//void
//LocaliserInput::Update(Eigen::Vector3d &observation, Eigen::Matrix3d &covariance, ros::Time stamp, std::string &source) {

//  if (std::isnan(observation[0]) ||
//      std::isnan(observation[1]) ||
//      std::isnan(observation[2])) {
//    ROS_INFO_STREAM_THROTTLE(1., "rejecting update due to NAN data");
//    return;
//  }

//  if (measured_speed > (2. / 3.6) && perform_update) {
//    perform_update(observation, covariance, stamp, source);
//  }
//}



