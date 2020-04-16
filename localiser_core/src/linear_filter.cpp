#include "linear_filter.hpp"

#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <boost/math/distributions/chi_squared.hpp>


LinearFilter::LinearFilter()
{
  confidence = 1.;
  initialised_filter = false;

  state << 0., 0., 0.;
  state_odom_only << 0., 0., 0.;
  state_var << 0., 0., 0., 0., 0., 0., 0., 0., 0.;

}

LinearFilter::~LinearFilter() {

}



//! Perform the optimisation
void
LinearFilter::AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Matrix2d& covariance, ros::Time stamp) {

  float delta_time = 0.01; //(stamp - prior_stamp).toSec();

  Eigen::Vector2d delta_state_change = motion * delta_time;
  state_odom_only = vehicle_model(state_odom_only, delta_state_change);

  if (publish_odometry) {
    Eigen::Matrix3d odom_uncertainty;

    odom_uncertainty << 0., 0., 0.,
        0., 0., 0.,
        0., 0., 0.;

    publish_odometry(state_odom_only, odom_uncertainty, stamp);
  }

  if (initialised) {

    Eigen::Matrix2d Q; //process noise
    Q << VELOCITY_NOISE, 0.,
        0., YAWRATE_NOISE;

    Q *= delta_time;
    Q = Q.array().square();

    predict(delta_state_change, Q);

    /*
       force the odom filter to use the absolute heading estimate - the odom source should be continuous
       in position but not in heading - the drifting of the baselink in the odom frame can only be
       in position. if there was an angular offset between the map->odom, this rotation
       could only be applied to the base_link directly
    */

    state_odom_only[2] = state[2];

    if (publish_map)
      publish_map(state, state_var, state_odom_only, stamp);
  }

  prior_motion = motion;
  prior_motion_covariance = covariance;
  prior_stamp = stamp;
}



void
LinearFilter::BoundHeading(const Eigen::Vector3d &current_state, Eigen::Vector3d &observation) {

  while (observation[2] < current_state[2] - M_PI) {
    observation[2] += (2.*M_PI);
  }
  while (observation[2] > current_state[2] + M_PI) {
    observation[2] -= (2.*M_PI);
  }
}



void
LinearFilter::update(const Eigen::VectorXd &observation,
                     const Eigen::MatrixXd &observation_noise,
                     const Eigen::MatrixXd &observation_matrix,
                     ros::Time stamp/*,
              uint64_t observation_timestamp*/) {
  /*
  Perform the update step of the EKF
  Conditions the state on observed values.

  NOTE: the current state should have applied the
        prediction step to the current time
  */

  //observation_matrix - fixed for a specific problem;
  Eigen::MatrixXd H = observation_matrix;   //Eigen::MatrixXd::Identity(3,3);

  Eigen::MatrixXd P = state_var;
  Eigen::MatrixXd R = observation_noise;

  // make the observations into column matricies
  // our observation model is simply the observation vector as we observe directly the values
  Eigen::VectorXd z = observation;

  Eigen::VectorXd zpred = H * state;

  Eigen::VectorXd x = state;

  // make x a column vector
  x = x.transpose();

  // v is the innovation
  Eigen::VectorXd v = z - zpred;

  //std::cout << "z, zpred, v " << z << std::endl << std::endl << zpred << std::endl << std::endl << v << std::endl << std::endl;

  // Calculate the KF (or EKF) update given the prior state [x,P], the
  // innovation v, the observe uncertainty R, and the (linearised)
  // observation model H. The result is calculated using Cholesky
  // factorisation, which is more numerically stable than a naive
  // implementation.
  //
  // From Matlab fns Adapted from code by Jose Guivant. Tim Bailey 2003.

  Eigen::MatrixXd Pht = P * H.transpose();


  //Eigen::Matrix3d factorised_variance = Pht.llt();
  //Eigen::Matrix3d innovation = factorised_variance.solve(v);
  Eigen::MatrixXd innovation = Pht.llt().matrixU().solve(v);

  auto statistic = innovation.array().square().sum();

  Eigen::MatrixXd tmp_mat = Pht.llt().matrixU();
/*  std::cout << "innovation " << innovation << std::endl << std::endl <<
        "P"  << P << std::endl << std::endl <<
         "Pht"  << Pht << std::endl << std::endl <<
         "Pht.llt().matrixU()" << tmp_mat <<  std::endl << std::endl <<
         //"Pht.llt().matrixU()" << Pht.llt().matrixU() <<  std::endl << std::endl <<
        "innovation square" << innovation.array().square() << std::endl << std::endl <<
        "innovation sum" << innovation.array().square().sum() << std::endl << std::endl;
  */
  boost::math::chi_squared distribution(3);

  double chi_confidence = boost::math::cdf(distribution, statistic);
  std::cout << "statistic " << std::setprecision(8) << statistic  << ", chi_confidence " << std::setprecision(8) << chi_confidence << std::endl;


  // Get the 95% confidence interval error ellipse
  double chisquare_val = 2.4477;

  // mean predicted values for the observed variables
  Eigen::MatrixXd observed_covariance = (H * P * H.transpose());
  Eigen::MatrixXd observed_variance = observed_covariance.diagonal().array().sqrt();
  //Eigen::VectorXd chi_95_percent = v.array() / (observed_variance * chisquare_val).array();
  Eigen::VectorXd chi_95 = (observed_variance * chisquare_val).array();
  chi_95 = chi_95.cwiseAbs();
  Eigen::VectorXd chi_95_percent = v.array() / chi_95.array();

  if (fabs(v(0)) < 2 and fabs(v(1)) < 2) {
    confidence = 0.95;
  }
  else if (fabs(v(0)) > 5 or fabs(v(1)) >5) {
    confidence = 0.99999;
  }
  else if (fabs(v(0)) > 25 or fabs(v(1)) > 25) {
    confidence = 0.999999999;
  }

  if (initialised_filter && chi_confidence > confidence) {
    std::cout << "REJECTING SAMPLE " << std::setprecision(8) << chi_confidence << " : " << std::setprecision(8) << confidence << std::endl << " v: " << v << std::endl;
    return;
  }

  initialised_filter = true;

  /*init to 1.
      if math.fabs(innovation[0]) < 2 and math.fabs(innovation[1]) < 2:
                self.filter.filter.confidence = 0.95
                rospy.loginfo('set confidence to 95%')
            elif math.fabs(innovation[0]) > 5 or math.fabs(innovation[1]) > 5:
                self.filter.filter.confidence = 0.99999
                rospy.loginfo('set confidence to 99.999%')

  */

  if (publish_statistics) {
    Eigen::Vector3d v_3d = v.array();
    Eigen::Vector3d chi_95_3d = chi_95.array();
    Eigen::Matrix3d covariance_3d;
    covariance_3d << observed_covariance;
    publish_statistics(v_3d, covariance_3d, chi_95_3d, stamp);
  }

//  std::cout << "covariance " << observed_covariance << std::endl;
//  std::cout << "variance diag" << observed_variance << std::endl;
//  std::cout << "innov " << v << std::endl;
//  std::cout << "chi_95 " << chi_95 << std::endl;
//  std::cout << "chi_95_percent " << chi_95_percent << std::endl;


  if (chi_95_percent.minCoeff() > 1. || chi_95_percent.minCoeff() < -1.) {
    std::cout << "CHI2 FAILED " << chi_95_percent << " : " << confidence << std::endl;
  }


  /*
  //if (chi_95_percent.minCoeff() > 1. || chi_95_percent.minCoeff() < -1.) {
//  if (observed_variance(0) < 1 && observed_variance(1) < 1) {
    if ((chi_95_percent(0) > 1. || chi_95_percent(0) < -1.) ||
        (chi_95_percent(1) > 1. || chi_95_percent(1) < -1.)) {
      ROS_INFO_STREAM_THROTTLE(1., "rejecting update outside chisquare test");
      return;
    }
//  }
*/
  //print 'EKF', chi_95_percent
  ////////self.consistency_history.append(chi_95_percent);

  Eigen::MatrixXd S = H * Pht + R;

  //assert(np.array_equal(S, H * P * H.transpose() + R));

  // TODO: test for success in cholesky factorisation
  // Cholesky factorisation - take upper triangle
  Eigen::MatrixXd Sc = S.llt().matrixU();

  //try {
  //     auto Sc = np.linalg.cholesky(S));
  ///////////////////// Eigen::Matrix3d Sci = S.ldlt().solve(Eigen::MatrixXd::Identity(3,3));
  //    Eigen::Matrix3d Sc = S.ldlt().solve(Eigen::MatrixXd::Identity(3,3));
  //}
  //  catch {
  //    print "Cholesky failed: adding noise"
  //    noise = np.matrix(np.diag((0.0005, 0.0005, 0.0005)));
  //    Sc = np.matrix(np.linalg.cholesky(S + noise));
  //  }


  // expecting upper triangle
  Eigen::MatrixXd Sci = Sc.inverse().triangularView<Eigen::Upper>();

  Eigen::MatrixXd Wc = Pht * Sci;
  Eigen::MatrixXd W = Wc * Sci.transpose();

  // Update the current state and covariance
  // x = x + W*v
  state = x + W * v;

  // P = P - Wc*Wc'
  state_var = P - Wc * Wc.transpose();
}


void
LinearFilter::predict(Eigen::VectorXd input_state, Eigen::MatrixXd input_state_variance) {

  //    Perform the prediction step of the EKF
  //    Update the mean and covariance, append the results to the state_history

  // apply transition model to predict current mean
  state = vehicle_model(state, input_state);

  // determine the transition matrix function
  Eigen::MatrixXd F = transition_matrix_fn(state, input_state);
  Eigen::MatrixXd G = jacobian_matrix_fn(state, input_state);

  Eigen::MatrixXd P = state_var;
  Eigen::MatrixXd Q = input_state_variance;

  // TODO: work out when there is no input?
  // Update the covariances depending on whether there is an input or not
  // if (min(G.shape) > 0)
  state_var = F * P * F.transpose() + G * Q * G.transpose();
  // else
  //   var = F * P * F.transpose();
}



void
PositionHeadingEKF::test_predict() {
  /*Test UKF() predict.*/

  // output generated by matlab to validate the calculations

  Eigen::Vector2d input_mean;
  input_mean << 0.13142498, 0.0052047;

  Eigen::Matrix2d input_variance;
  input_variance << 1.60000000e-03, 0.00000000e+00,
      0.00000000e+00, 9.27917724e-08;

  Eigen::Vector3d prior_mean;
  prior_mean << 0., 0., 0.;

  Eigen::Matrix3d prior_variance;
  prior_variance << 100., 0., 0.,
      0., 100., 0.,
      0., 0., 0.81;

  state = prior_mean;
  state_var = prior_variance;

  // fill the state history with the initial guess
  //ukf.init(StateEstimate(prior_mean, prior_variance))

  predict(input_mean, input_variance);

  // expected output determined by running the input through matlab
  Eigen::Matrix3d expected_variance;
  expected_variance << 1.000016004094332e+02, 0.000000521102872e+02, -0.000006415931298e+02,
      0.000000521102872e+02, 1.000082323026614e+02, -0.000816583452286e+02,
      -0.000006415931298e+02, -0.000816583452286e+02, 0.008100000231979e+02;


/*
  Eigen::Vector3d expected_mean = vehicle_model(prior_mean, input_mean);
    std::cout << "predict test" << std::endl << state << std::endl << std::endl
        << expected_mean << std::endl << std::endl
        << state_var << std::endl << std::endl
        << expected_variance << std::endl << std::endl;
*/
  // performing a comparison to the EKF, need to run this through matlab to determine the proper values for the UKF

  // compare the expected output variance (from matlab) with the calculated variance from the filter
  //self.assertTrue(numpy.allclose(expected_variance, ukf.state_history[-1].var, 1e-01, 1e-01))

  // compare the expected output mean (from matlab) with the calculated mean from the filter
  //self.assertTrue(numpy.allclose(expected_mean, ukf.state_history[-1].mean, 1e-01, 1e-01))
}



void
PositionHeadingEKF::test_update() {
  /* Test UKF() update. */
  // output generated by matlab to validate the calculations

  Eigen::Vector3d prior_mean;
  prior_mean << 0.04301946, -0.00011447, 0.00532197;


  Eigen::Matrix3d prior_variance;
  prior_variance << 1.00001600e+02, -8.05790926e-07, -2.78170313e-04,
      -8.05790926e-07, 1.00001499e+02, -3.48447798e-02,
      -2.78170313e-04, -3.48447798e-02, 8.10000023e-01;

  state = prior_mean;
  state_var = prior_variance;


  Eigen::Matrix3d H = Eigen::MatrixXd::Identity(3,3);

  Eigen::Matrix3d R;
  R  << 1.00000000e-06, 0.00000000e+00, 0.00000000e+00,
      0.00000000e+00, 1.00000000e-06, 0.00000000e+00,
      0.00000000e+00, 0.00000000e+00, 9.27917724e-06;

  Eigen::Vector3d z;
  z << 4.30656587e-02, 5.81005984e-05, -1.56530542e+00;

  update(z, R, H, ros::Time::now());

  // expected output determined by running the input through matlab
  Eigen::Matrix3d expected_variance;
  expected_variance << 0.099999994063182e-05, -0.000000000000000e-05, -0.000000000003182e-05,
      -0.000000000000000e-05, 0.099999998326439e-05, -0.000000000398986e-05,
      -0.000000000003182e-05, -0.000000000398986e-05, 0.927907093917213e-05;

  Eigen::Vector3d expected_mean;
  expected_mean << 0.043065658704932, 0.000058101272323, -1.565287427184605;


  std::cout << "update test" << std::endl << state << std::endl << std::endl
            << expected_mean << std::endl << std::endl
            << state_var << std::endl << std::endl
            << expected_variance << std::endl << std::endl;

  // compare the expected output variance (from matlab) with the calculated variance from the filter
  //self.assertTrue(numpy.allclose(expected_variance, ukf.state_history[-1].var))

  // compare the expected output mean (from matlab) with the calculated mean from the filter
  //self.assertTrue(numpy.allclose(expected_mean, ukf.state_history[-1].mean))
}



Eigen::Vector3d
PositionHeadingEKF::vehicle_model(const Eigen::Vector3d &mean, const Eigen::Vector2d &input_state){
  /*
  Vehicle transition model.

  Transition function for the vehicle model

      Args:
      mean (array): state [x (meters),
      y (meters),
      heading (radians)]

  input_state (array): [delta_position (meters),
      delta_heading (radians)]
  */

  Eigen::Vector3d posterior;

  // av_heading is the average heading for the time period
  double av_heading = mean[2] + (input_state[1] / 2.);

  posterior[0] = mean[0] + input_state[0] * cos(av_heading);
  posterior[1] = mean[1] + input_state[0] * sin(av_heading);
  posterior[2] = mean[2] + input_state[1];

  return posterior;
}



Eigen::VectorXd
PositionHeadingEKF::observation_model(const Eigen::VectorXd state, const Eigen::VectorXd observation){
  // returns the observation model
  return observation;
}



Eigen::Matrix3d
PositionHeadingEKF::transition_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state) {
  /*
    generate covariance transition matrix
    mean[x, y, theta]
    input_state[delta_p, delta_theta]
  */
  Eigen::MatrixXd transition_matrix = Eigen::MatrixXd::Identity(3, 3);

  // parameters used in the transition matrix
  double delta_p = input_state[0];
  double delta_theta = input_state[1];
  double theta = mean[2];

  double av_heading = theta + (delta_theta / 2.0);

  transition_matrix(0,2) = -1.0 * delta_p * sin(av_heading);
//# transition_matrix[1][2] = -1.0 * delta_p * math.cos(av_heading)
  transition_matrix(1,2) = delta_p * cos(av_heading);

  return transition_matrix;
}



Eigen::MatrixXd
PositionHeadingEKF::jacobian_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state) {
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




void
PositionHeadingEKF::AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time stamp) {

  if (!initialised) {
    if (prior_motion[0] > MINIMUM_INITIALISATION_SPEED) {
      state = observation;
      state_var = covariance;
      initialised = true;
      prior_stamp = stamp;
    }
  }
  else {
    // The observations are generally older than the latest relative motion estimate
    //float delta_time = (stamp - prior_stamp).toSec();
    //predict(motion * delta_time, covariance);

    BoundHeading(state, observation);

    // Override covariance with fixed values
    Eigen::Matrix3d R; // observation noise

    R << POSITION_ERROR, 0., 0.,
        0., POSITION_ERROR, 0.,
        0., 0., HEADING_ERROR;

    R = R.array().square();

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(3,3);

    //update(observation, covariance);
    update(observation, R, H, stamp);
  }
}







Eigen::Vector3d
PositionOnlyEKF::vehicle_model(const Eigen::Vector3d &mean, const Eigen::Vector2d &input_state){
  /*
  Vehicle transition model.

  Transition function for the vehicle model

      Args:
      mean (array): state [x (meters),
      y (meters),
      heading (radians)]

  input_state (array): [delta_position (meters),
      delta_heading (radians)]
  */

  Eigen::Vector3d posterior;

  // av_heading is the average heading for the time period
  double av_heading = mean[2] + (input_state[1] / 2.);

  posterior[0] = mean[0] + input_state[0] * cos(av_heading);
  posterior[1] = mean[1] + input_state[0] * sin(av_heading);
  posterior[2] = mean[2] + input_state[1];

  return posterior;
}



Eigen::VectorXd
PositionOnlyEKF::observation_model(const Eigen::VectorXd state, const Eigen::VectorXd observation){
  // returns the observation model
  Eigen::VectorXd partial_observation;
  partial_observation << observation[0], observation[1];
  return partial_observation;
}



Eigen::Matrix3d
PositionOnlyEKF::transition_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state) {
  /*
    generate covariance transition matrix
    mean[x, y, theta]
    input_state[delta_p, delta_theta]
  */
  Eigen::MatrixXd transition_matrix = Eigen::MatrixXd::Identity(3, 3);

  // parameters used in the transition matrix
  double delta_p = input_state[0];
  double delta_theta = input_state[1];
  double theta = mean[2];

  double av_heading = theta + (delta_theta / 2.0);

  transition_matrix(0,2) = -1.0 * delta_p * sin(av_heading);
//# transition_matrix[1][2] = -1.0 * delta_p * math.cos(av_heading)
  transition_matrix(1,2) = delta_p * cos(av_heading);

  return transition_matrix;
}



Eigen::MatrixXd
PositionOnlyEKF::jacobian_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state) {
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


void
PositionOnlyEKF::AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time stamp) {

  if (!initialised) {
    if (prior_motion[0] > MINIMUM_INITIALISATION_SPEED) {
      state = observation;
      state_var = covariance;
      initialised = true;
      prior_stamp = stamp;
    }
  }
  else {
    // The observations are generally older than the latest relative motion estimate
    //float delta_time = (stamp - prior_stamp).toSec();
    //predict(motion * delta_time, covariance);

    BoundHeading(state, observation);

    // Override covariance with fixed values
    Eigen::Matrix2d R; // observation noise

    R << POSITION_ERROR, 0.,
        0., POSITION_ERROR;

    R = R.array().square();

    Eigen::MatrixXd H = Eigen::MatrixXd(2,3);
    H << 1., 0., 0.,
         0., 1., 0.;

    //update(observation, covariance);
    update(observation, R, H, stamp);
  }
}

