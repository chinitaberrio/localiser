#include "motion_model.h"

MotionModel::MotionModel()
{

}

void MotionModel::receive_pitch(double pitch, double variance, ros::Time stamp) {
  measured_pitch = pitch;
  // todo: add roll, output horizon tf to bag
}

void MotionModel::receive_speed(double speed, double variance, ros::Time stamp) {
  measured_speed = speed;
}

void MotionModel::receive_yaw_rate(double yaw_rate, double variance, ros::Time stamp) {
  measured_yaw_rate = yaw_rate;
}



void MotionModel::calculate_pose_increment(ros::Time stamp) {
  if (signal_prediction) {

    double speed_horizontal = measured_speed * cos(measured_pitch);

    Eigen::Vector2d motion;
    motion << speed_horizontal, measured_yaw_rate;

    float delta_time = 0.01; //(stamp - prior_stamp).toSec();

    Eigen::Vector2d delta_state_change = motion * delta_time;

    // not using covariance parsed from rosmsg, as rosmsg covariance is a fixed small value, we don't trust it
    // using our own fixed larger covariance
    Eigen::Matrix2d motion_noise << VELOCITY_NOISE, 0.,
                                    0., YAWRATE_NOISE;

    motion_noise *= delta_time;
    motion_noise = predict_step->motion.noise.array().square();



    //    Perform the prediction step of the EKF

    // apply vehicle_model to predict current mean
    Eigen::Vector3d increment_mean = vehicle_model(delta_state_change[0], delta_state_change[1]);

    // determine the transition matrix function
    Eigen::MatrixXd G = jacobian_matrix_fn(increment_mean, delta_state_change);

    Eigen::MatrixXd Q = motion_noise;

   // TODO: work out when there is no input?
   // Update the covariances depending on whether there is an input or not
   // if (min(G.shape) > 0)
    increment_covariance = G * Q * G.transpose();


    signal_prediction(increment_mean, increment_covariance, stamp);
  }
}


Eigen::Vector3d
MotionModel::vehicle_model(double distance_travelled, double delta_heading){

  Eigen::Vector3d posterior_in_prior_frame;

  // av_heading is the average heading for the time period
  double av_heading = delta_heading / 2.;

  posterior_in_prior_frame[0] =  distance_travelled * cos(av_heading);
  posterior_in_prior_frame[1] =  distance_travelled * sin(av_heading);
  posterior_in_prior_frame[2] =  delta_heading;

  return posterior_in_prior_frame;
}


