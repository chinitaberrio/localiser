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

    Eigen::Vector2d motion;
    Eigen::Matrix2d motion_noise;
    // if speed is 0, do not project it to horizontal plane, assign linear and angular velocity 0.0
    if(abs(measured_speed) < 0.00001){
        motion << 0.0, 0.0;
        motion_noise << 0., 0.,
                        0., 0.;
    }else{
        double speed_horizontal = measured_speed * cos(measured_pitch);
        motion << speed_horizontal, measured_yaw_rate;

        // not using covariance parsed from rosmsg, as rosmsg covariance is a fixed small value, we don't trust it
        // using our own fixed larger covariance
        motion_noise << VELOCITY_NOISE, 0.,
                        0., YAWRATE_NOISE;
    }


    float delta_time = 0.01; //(stamp - prior_stamp).toSec();

    Eigen::Vector2d delta_state_change = motion * delta_time;

//    std::cout << "dxy: " << delta_state_change(0) << " dyaw: " << delta_state_change(1) << std::endl;


    motion_noise *= delta_time;
    motion_noise = motion_noise.array().square();



    //    Perform the prediction step of the EKF

    // apply vehicle_model to predict current mean
    Eigen::Vector3d increment_mean = vehicle_model(delta_state_change[0], delta_state_change[1]);

    // determine the transition matrix function
    Eigen::MatrixXd G = jacobian_matrix_fn(increment_mean, delta_state_change);

    Eigen::MatrixXd Q = motion_noise;

   // TODO: work out when there is no input?
   // Update the covariances depending on whether there is an input or not
   // if (min(G.shape) > 0)
    Eigen::Matrix3d increment_covariance = G * Q * G.transpose();

    signal_prediction(increment_mean, increment_covariance, stamp);
  }
}


Eigen::Vector3d
MotionModel::vehicle_model(double distance_travelled, double delta_heading){

  Eigen::Vector3d posterior_in_prior_frame;

  // polar coordinate of new position(distance_travelled, av_heading = delta_heading / 2), heading = delta_heading
  double av_heading = delta_heading / 2.;

  // convert polar position to cartesian position
  posterior_in_prior_frame[0] =  distance_travelled * cos(av_heading);
  posterior_in_prior_frame[1] =  distance_travelled * sin(av_heading);
  posterior_in_prior_frame[2] =  delta_heading;

//  std::cout << "dx: " << posterior_in_prior_frame(0) << " dy: " << posterior_in_prior_frame(1) << std::endl;


  return posterior_in_prior_frame;
}

Eigen::MatrixXd
MotionModel::jacobian_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state) {
  /*
    generate jacobian matrix
        mean[x, y, theta]
        input_state[delta_p, delta_theta]

        This is the linearisation of the vehicle model for the input parameters
  */

  Eigen::MatrixXd jacobian_mat = Eigen::MatrixXd::Zero(3, 2);

  // parameters used in the jacobian
  auto delta_p = input_state[0];
  auto delta_theta = input_state[1];
  auto theta = mean[2];

  // 0.5 is included as d/dx cos(0.5x) = -0.5sin(0.5x)

  // partial derivatives
  // delta_p w.r.t. x
  jacobian_mat(0,0) = cos(theta + 0.5 * delta_theta);
  //  print ("recalculate the off diagonal jacobians")
  // delta_theta w.r.t. x
  jacobian_mat(0,1) = (-0.5 * delta_p * cos(theta) * sin(0.5 * delta_theta) -
                       0.5 * delta_p * sin(theta) * cos(0.5 * delta_theta));

  // delta_p w.r.t. y
  // jacobian_mat[1][0] = -1.0 * math.sin(theta + 0.5 * delta_theta)
  jacobian_mat(1,0) = sin(theta + 0.5 * delta_theta);

  // delta_theta w.r.t. y
  jacobian_mat(1,1) = (-0.5 * delta_p * cos(theta) * cos(0.5 * delta_theta) +
                       0.5 * delta_p * sin(theta) * sin(0.5 * delta_theta));

  // delta_p w.r.t. theta
  jacobian_mat(2,0) = 0;

  // delta_theta w.r.t. theta
  jacobian_mat(2,1) = 0.5;

  return jacobian_mat;
}
