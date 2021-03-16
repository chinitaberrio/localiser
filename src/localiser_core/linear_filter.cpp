#include "linear_filter.h"


#include <algorithm>

#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

//#include <Eigen/Core>
//#include <Eigen/StdVector>

#include <boost/math/distributions/chi_squared.hpp>


LinearFilter::LinearFilter()
{
   state_odom_only << 0., 0., 0.;
}


void
LinearFilter::BoundHeading(const Eigen::Vector3d &current_state, Eigen::Vector3d &observation) {

//    std::cout << "observation[2] " << observation[2] << " current_state[2] " << current_state[2];
  while (observation[2] < current_state[2] - M_PI) {
    observation[2] += (2.*M_PI);
  }
  while (observation[2] > current_state[2] + M_PI) {
    observation[2] -= (2.*M_PI);
  }

//  std::cout << " bound observation[2] " << observation[2] << "\n";
}



bool
PredictStep::condition(StateEstimate &prior) {


  //    Perform the prediction step of the EKF
  //    Update the mean and covariance, append the results to the state_history

  // apply transition model to predict current mean
  posterior.mean = motion_model(prior.mean, motion.mean);

  // determine the transition matrix function
  Eigen::MatrixXd F = transition_matrix_fn(posterior.mean, motion.mean);
  Eigen::MatrixXd G = jacobian_matrix_fn(posterior.mean, motion.mean);

  Eigen::MatrixXd P = prior.covariance;
  Eigen::MatrixXd Q = motion.noise;

  // TODO: work out when there is no input?
  // Update the covariances depending on whether there is an input or not
  // if (min(G.shape) > 0)
  posterior.covariance = F * P * F.transpose() + G * Q * G.transpose();
  // else
  //   var = F * P * F.transpose();

//  std::cout << "motion noise:\n" << Q
//            << "\nprior noise:\n" << P
//            << "\nposterior noise:\n" << posterior.covariance
//            << "\n\n";

  // todo: are there any conditions where the predict should fail ?
  return true;
}


bool
UpdateStep::CalculateConsensus(float consensus_window, std::deque<std::shared_ptr<ConditionedState>> &states) {

  if(!prev || fabs((stamp - prev->stamp).toSec()) > consensus_window) {
    // cannot determine consensus without recent samples
    std::cout << "no previous sample" << std::endl;
    valid_individual = false;
    valid_sequence = false;

    return true;
  }


  if (prev && prev->prev) {
    //estimate angle change

  }
  // calculate the discrepancy in speed since the last sample

  float distance_update = sqrt(pow(observation.mean(0) - prev->observation.mean(0), 2) +
                                     pow(observation.mean(1) - prev->observation.mean(1), 2));

  float distance_predict = 0;

  // set this value
  //float speed_discrepancy;

  auto current_observation = std::find(states.begin(), states.end(), prev);
  auto end_observation = std::find(states.begin(), states.end(), prev->next);

//  float distance_predict = 0;

  uint16_t counter = 0;
  if (current_observation != states.end()) {
    while (++current_observation != end_observation) {
      counter++;
      //std::shared_ptr<ConditionedState> state = *current_observation;
      auto predict_step = std::dynamic_pointer_cast<PredictStep>(*current_observation);
      if (!predict_step) {
        std::cout << "why not ?" << std::endl;
        continue;
      }
      distance_predict += predict_step->motion.mean(0);
    }
  }
  else {
    std::cout << "couldn't find iterator" << std::endl;
  }

  speed_discrepancy = fabs((distance_update - distance_predict) / distance_update);


  std::list<std::shared_ptr<UpdateStep>> test_updates;

  float square_error_sum = 0.;
  auto back_iterate = prev->next;
  while (back_iterate && (stamp - back_iterate->stamp).toSec() < consensus_window) {
    test_updates.push_front(back_iterate);
    square_error_sum += pow(speed_discrepancy, 2);
    back_iterate = back_iterate->prev;
  }

  if (back_iterate) {
    std::cout << "distance " << speed_discrepancy << ": " << distance_update << ", "
              << distance_predict << " sum^2 " << square_error_sum
              << " from " << test_updates.size() << " samples" << std::endl;

  }

//  if (speed_discrepancy > 0.09 /*|| square_error_sum > 0.02*/) {
//  if (speed_discrepancy > 9 /*|| square_error_sum > 0.02*/) {

//    std::cout << "REJECT" << std::endl;
//    valid_flag = false;
//    valid_individual = false;

//  }



  // determine using the last N UpdateStep consensus values (calculate mean square error) whether it is over the threshold,
  //  then set the valid flag.

  // calculate the previous values of the angle and use this also - recalc the mean square error for prior window
  //   and potentially invalidate old samples

  // rerun the filter for the window using the update to bypass invalid samples.
}


bool
UpdateStep::condition(StateEstimate &prior/*,
                     Observation &observation,
                     StateEstimate &posterior*/) {

  if (!valid_individual) {
    posterior.mean = prior.mean;
    posterior.covariance = prior.covariance;
    return false;
  }
  // todo: need to find where this goes
//  double confidence = 1.;

/*LinearFilterAbsoluteSample &prior,
                 const Eigen::VectorXd &observation,
                 const Eigen::MatrixXd &observation_noise,
                 LinearFilterAbsoluteSample &posterior,
                 const Eigen::MatrixXd &observation_matrix,
                 ros::Time stamp*//*,
              uint64_t observation_timestamp*///) {
  /*
  Perform the update step of the EKF
  Conditions the state on observed values.

  NOTE: the current state should have applied the
        prediction step to the current time
  */

  //observation_matrix - fixed for a specific problem;
  Eigen::MatrixXd H = observation.observation_matrix;   //Eigen::MatrixXd::Identity(3,3);

  Eigen::MatrixXd P = prior.covariance;
  Eigen::MatrixXd R = observation.noise;

  // make the observations into column matricies
  // our observation model is simply the observation vector as we observe directly the values
  Eigen::VectorXd z = observation.mean;

  Eigen::VectorXd zpred = H * prior.mean;

  Eigen::VectorXd x = prior.mean;

  // make x a column vector
  x = x.transpose();

  // v is the innovation
  Eigen::VectorXd v = z - zpred;

  //std::cout << "x, P, H " << x << std::endl << std::endl << P << std::endl << std::endl << H << std::endl << std::endl;
  //std::cout << "z, zpred, v " << z << std::endl << std::endl << zpred << std::endl << std::endl << v << std::endl << std::endl;
//  std::cout << "z, zpred, v " << z << std::endl << std::endl << zpred << std::endl << std::endl << v << std::endl;

  // Calculate the KF (or EKF) update given the prior state [x,P], the
  // innovation v, the observe uncertainty R, and the (linearised)
  // observation model H. The result is calculated using Cholesky
  // factorisation, which is more numerically stable than a naive
  // implementation.
  //
  // From Matlab fns Adapted from code by Jose Guivant. Tim Bailey 2003.

  // Pht: P matrix times H matrix's transpose
  Eigen::MatrixXd Pht = P * H.transpose();


  //Eigen::Matrix3d factorised_variance = Pht.llt();
  //Eigen::Matrix3d innovation = factorised_variance.solve(v);
  Eigen::MatrixXd innovation = Pht.llt().matrixU().solve(v);


  innovation(2,0) = 0.;
  auto statistic = innovation.array().square().sum();
  if(std::isnan(statistic))
      statistic = 10000.;

  Eigen::MatrixXd tmp_mat = Pht.llt().matrixU();
//  std::cout /*<< "v\n" << v
//            << "\ninnovation\n" << innovation*/
//            << "P\n" << P
//            << std::endl; //<< std::endl <<
 /*       "P"  << P << std::endl << std::endl <<
         "Pht"  << Pht << std::endl << std::endl <<
         "Pht.llt().matrixU()" << tmp_mat <<  std::endl << std::endl <<
         //"Pht.llt().matrixU()" << Pht.llt().matrixU() <<  std::endl << std::endl <<
        "innovation square" << innovation.array().square() << std::endl << std::endl <<
        "innovation sum" << innovation.array().square().sum() << std::endl << std::endl;
  */
  boost::math::chi_squared distribution(3);
//  std::cout << "statistic\n" << std::setprecision(4) << statistic
//            << std::endl;

  double chi_confidence = boost::math::cdf(distribution, statistic);


  // Get the 95% confidence interval error ellipse
  double chisquare_val = 2.4477;

  // mean predicted values for the observed variables
  Eigen::MatrixXd observed_covariance = (H * P * H.transpose());
  Eigen::MatrixXd observed_variance = observed_covariance.diagonal().array().sqrt();
  //Eigen::VectorXd chi_95_percent = v.array() / (observed_variance * chisquare_val).array();
  Eigen::VectorXd chi_95 = (observed_variance * chisquare_val).array();
  chi_95 = chi_95.cwiseAbs();
  Eigen::VectorXd chi_95_percent = v.array() / chi_95.array();


  if (fabs(v(0)) < 0.5 and fabs(v(1)) < 0.5) {
    next_confidence = 0.9999;
  }
//  else if (fabs(v(0)) < 2 or fabs(v(1)) < 2) {
//    confidence = 0.999;
//  }
  else {
//  else if (fabs(v(0)) > 5 or fabs(v(1)) > 5) {
    next_confidence = 0.999999999;
  }


  if (/*initialised_filter && */chi_confidence > confidence) {
//    std::cout << "REJECTING SAMPLE "
//              << std::setprecision(10) << chi_confidence << " : "
//              << std::setprecision(10) << confidence << std::endl;
//              << " v: " << v(0)
//              << " z: " << z(0)
//              << " zpred: " << zpred(0)
//              << std::endl;
    return false;
  }

//  std::cout << "INCORPORATING SAMPLE "
//            << std::setprecision(10) << chi_confidence << " : "
//            << std::setprecision(10) << confidence << std::endl;


//  initialised_filter = true;

  /*init to 1.
      if math.fabs(innovation[0]) < 2 and math.fabs(innovation[1]) < 2:
                self.filter.filter.confidence = 0.95
                rospy.loginfo('set confidence to 95%')
            elif math.fabs(innovation[0]) > 5 or math.fabs(innovation[1]) > 5:
                self.filter.filter.confidence = 0.99999
                rospy.loginfo('set confidence to 99.999%')

  */
// todo: publish the stats
//  if (publish_statistics) {
    v_3d = v.array();
    chi_95_3d = chi_95.array();
    chi_95_3d[0] = chi_confidence;
    covariance_3d << observed_covariance;
//todo ///////    publish_statistics(v_3d, covariance_3d, chi_95_3d, observation.stamp);
//  }

//  std::cout << "covariance " << observed_covariance << std::endl;
//  std::cout << "variance diag" << observed_variance << std::endl;
//  std::cout << "innov " << v << std::endl;
//  std::cout << "chi_95 " << chi_95 << std::endl;
//  std::cout << "chi_95_percent " << chi_95_percent << std::endl;


  if (chi_95_percent.minCoeff() > 1. || chi_95_percent.minCoeff() < -1.) {
 //   std::cout << "CHI2 FAILED " << chi_95_percent << " : " << confidence << std::endl;
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
  // Sc: S matrix's cholesky
  Eigen::MatrixXd Sc = S.llt().matrixU();

//  try {
//       auto Sc = np.linalg.cholesky(S));
//  /////////////////// Eigen::Matrix3d Sci = S.ldlt().solve(Eigen::MatrixXd::Identity(3,3));
//      Eigen::Matrix3d Sc = S.ldlt().solve(Eigen::MatrixXd::Identity(3,3));
//  }
//    catch {
//      print "Cholesky failed: adding noise"
//      noise = np.matrix(np.diag((0.0005, 0.0005, 0.0005)));
//      Sc = np.matrix(np.linalg.cholesky(S + noise));
//    }


  // expecting upper triangle
  // Sci: Sc matrix's inverse
  Eigen::MatrixXd Sci = Sc.inverse().triangularView<Eigen::Upper>();

  // Wc: kalman gain K
  Eigen::MatrixXd Wc = Pht * Sci;
  Eigen::MatrixXd W = Wc * Sci.transpose();

  // Update the current state and covariance
  // x = x + W*v
  posterior.mean = x + W * v;

  // P = P - Wc*Wc'
  posterior.covariance = P - Wc * Wc.transpose();

//  Eigen::MatrixXd Wc2 = Wc * Wc.transpose();

//  std::cout << "Wc2\n " << Wc2

//            << std::endl;

  return true;
}



void
PositionHeadingEKF::test_predict() {
  /*Test UKF() predict.*/

  // output generated by matlab to validate the calculations

  StateEstimate prior, posterior;
  RelativeMotion motion;

  //Eigen::Vector2d input_mean;
  motion.mean << 0.13142498, 0.0052047;

  Eigen::Matrix2d input_variance;
  motion.noise << 1.60000000e-03, 0.00000000e+00,
      0.00000000e+00, 9.27917724e-08;

  //Eigen::Vector3d prior_mean;
  prior.mean << 0., 0., 0.;

  //Eigen::Matrix3d prior_variance;
  prior.covariance << 100., 0., 0.,
      0., 100., 0.,
      0., 0., 0.81;

  //state = prior_mean;
  //state_var = prior_variance;

  // fill the state history with the initial guess
  //ukf.init(StateEstimate(prior_mean, prior_variance))

  //////predict(prior, motion, posterior);

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

  StateEstimate prior, posterior;

  prior.mean << 0.04301946, -0.00011447, 0.00532197;

  prior.covariance << 1.00001600e+02, -8.05790926e-07, -2.78170313e-04,
      -8.05790926e-07, 1.00001499e+02, -3.48447798e-02,
      -2.78170313e-04, -3.48447798e-02, 8.10000023e-01;

//  state = prior_mean;
//  state_var = prior_variance;

  Observation observation;

  observation.observation_matrix = Eigen::MatrixXd::Identity(3,3);


  //Eigen::Matrix3d R;
  observation.noise  << 1.00000000e-06, 0.00000000e+00, 0.00000000e+00,
      0.00000000e+00, 1.00000000e-06, 0.00000000e+00,
      0.00000000e+00, 0.00000000e+00, 9.27917724e-06;

  //Eigen::Vector3d z;
  observation.mean << 4.30656587e-02, 5.81005984e-05, -1.56530542e+00;

  //observation.stamp = ros::Time::now();

  /////update(prior, observation, posterior);

  // expected output determined by running the input through matlab
  Eigen::Matrix3d expected_variance;
  expected_variance << 0.099999994063182e-05, -0.000000000000000e-05, -0.000000000003182e-05,
      -0.000000000000000e-05, 0.099999998326439e-05, -0.000000000398986e-05,
      -0.000000000003182e-05, -0.000000000398986e-05, 0.927907093917213e-05;

  Eigen::Vector3d expected_mean;
  expected_mean << 0.043065658704932, 0.000058101272323, -1.565287427184605;

//  std::cout << "update test" << std::endl
//            << expected_mean << std::endl << std::endl
//            << posterior.mean << std::endl << std::endl
//            << expected_variance << std::endl << std::endl
//            << posterior.covariance << std::endl << std::endl;

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

//  std::cout <<"x: "<< posterior[0] <<" y: "<< posterior[1] <<" dxy: "<< input_state[0]<< " in KF\n";

  return posterior;
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
PositionHeadingEKF::AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time &stamp, std::string &source) {

    // if vehicle is reverse driving, vehicle heading should be opposite of differential GPS heading
    // find substring "gnss" in update source. if return is not end of string
    size_t found_gnss = source.find("gnss");
    size_t found_rtkFix = source.find("rtkFix");

//    std::cout << "previous_speed: " << previous_speed << std::endl;

    if(found_gnss != std::string::npos  && previous_speed < 0.){
        observation(2) = observation(2) + M_PI; // 180 degrees off with true heading
    }

  if (std::isnan(observation(2)) || (abs(previous_speed) < 1. && found_rtkFix == std::string::npos)
          || (abs(previous_speed) < 0.5 && found_rtkFix != std::string::npos)) {

    observation(2) = 0.;

    if (initialised) {
        if (abs(previous_speed) < 1.){
            last_source = std::make_shared<std::string>(source + "-slow");
            ROS_INFO_STREAM_THROTTLE(1, "reject observation due to low speed" );
        }else{
            last_source = std::make_shared<std::string>(source);
            ROS_INFO_STREAM_THROTTLE(1, "reject observation due to no GPS fix" );
        }

        // Add directly
        std::shared_ptr<UpdateStep> update_step = std::make_shared<UpdateStep>();

        update_step->posterior.mean = states.back()->posterior.mean;
        update_step->posterior.covariance = states.back()->posterior.covariance;

//        std::cout << "posterior.mean:\n" << update_step->posterior.mean;

        update_step->observation.mean = observation;
        update_step->observation.noise = covariance;

        update_step->valid_individual = false;
        update_step->valid_sequence = false;
        update_step->stamp = stamp;

        // connect update_step to linked list of update steps
        if (last_update) {
          last_update->next = update_step;
          update_step->prev = last_update;

          update_step->confidence = last_update->next_confidence;
          update_step->next_confidence = last_update->next_confidence;
        }

        states.push_back(update_step);
        last_update = update_step;
    }
    for (auto &signal_stats : signal_statistics) {
        Eigen::Vector3d zero_vector;
        zero_vector.setZero();
        Eigen::Matrix3d zero_matrix;
        zero_matrix.setZero();
        signal_stats(zero_vector, zero_vector, zero_matrix, zero_vector, stamp, *last_source);
    }
    for (auto &signal_stats : signal_update_stats) {
        Eigen::Vector3d zero_vector;
        zero_vector.setZero();
        Eigen::Matrix3d zero_matrix;
        zero_matrix.setZero();
        signal_stats(observation, zero_vector, covariance, zero_vector, stamp, *last_source);
    }

//    std::cout << "update source: " << *last_source << std::endl;
    return;
  }

  if (!initialised) {


      // Add directly, this is the initialisation of the filter, so it is definitely considered valid
      std::shared_ptr<UpdateStep> update_step = std::make_shared<UpdateStep>();
      states.push_back(update_step);

      update_step->posterior.mean = observation;
      update_step->posterior.covariance = covariance;

      update_step->observation.mean = observation;
      update_step->observation.noise = covariance;

      update_step->valid_individual = true;
      update_step->valid_sequence = false;
      update_step->stamp = stamp;

      initialised = true;
      last_source = std::make_shared<std::string>(source);
      ROS_WARN("PositionHeadingEKF initialised");

  }
  else {
    // The observations are generally older than the latest relative motion estimate
//    float delta_time = (stamp - prior_stamp).toSec();
//    predict(motion * delta_time, covariance);
    StateEstimate &prior = states.back()->posterior;
    std::shared_ptr<UpdateStep> update_step = std::make_shared<UpdateStep>();
    update_step->observation.mean = observation;
    update_step->observation.noise = covariance;

    //obs->prior = states.back().posterior;
    BoundHeading(prior.mean, update_step->observation.mean);

    update_step->valid_individual = true;
    update_step->valid_sequence = false;
    update_step->stamp = stamp;



//    Eigen::MatrixXd H = Eigen::MatrixXd(2,3);
    update_step->observation.observation_matrix = Eigen::MatrixXd(3,3);
    update_step->observation.observation_matrix  << 1., 0., 0.,
                                                    0., 1., 0.,
                                                    0., 0., 1.;


    // connect update_step to linked list of update steps
    if (last_update) {
      last_update->next = update_step;
      update_step->prev = last_update;
    }

//    update_step->CalculateConsensus(1.0, states);

    // always incorporate rtkFix(do not need to pass speed threshold)
    // if not rtkFix, use confidence determined from last update
    if(found_rtkFix != std::string::npos){
        update_step->confidence = 1.;
    }else if (last_update) {
        update_step->confidence = last_update->next_confidence;
    }


    if (update_step->condition(prior)) {
      last_source = std::make_shared<std::string>(source);
      states.push_back(update_step);
    }
    else {
      std::string source_bad = source + "-reject";
      last_source = std::make_shared<std::string>(source_bad);
    }

    for (auto &signal_stats : signal_statistics) {
      signal_stats(update_step->posterior.mean, update_step->v_3d, update_step->covariance_3d,
                   update_step->chi_95_3d, update_step->stamp, *last_source);
    }

    for (auto &signal_stats : signal_update_stats) {
      signal_stats(observation, update_step->v_3d, covariance,
                   update_step->chi_95_3d, update_step->stamp, *last_source);
    }

//    std::cout << "update source: " << *last_source << std::endl;

    last_update = update_step;

  }

}


//! Perform the optimisation
void
PositionHeadingEKF::AddRelativeMotion(Eigen::Vector3d& increment, Eigen::Matrix3d& increment_cov, ros::Time &stamp) {

    // TODO: reject all predict and updates if time stamp is earlier than last time stamp. reject out of sequence msgs.

    Eigen::Vector2d delta_state_change;
    delta_state_change << std::hypot(increment[0], increment[1]), increment[2];
    // if x is negative, vehcle is going in reverse, speed is assigned negative sign
    if (increment[0] < 0.)
        delta_state_change(0) = -delta_state_change(0);

    previous_speed = 100 * delta_state_change[0];
    state_odom_only = vehicle_model(state_odom_only, delta_state_change);

    Eigen::Matrix3d odom_uncertainty;
    odom_uncertainty.setZero();
    if (!signal_odom_state.empty()) {
        for(auto &signal_odom : signal_odom_state){
            signal_odom(state_odom_only, odom_uncertainty, stamp);
        }
    }


  if (initialised) {

    StateEstimate &prior = states.back()->posterior;

    std::shared_ptr<PredictStep> predict_step = std::make_shared<PredictStep>();

    using std::placeholders::_1;
    using std::placeholders::_2;
    predict_step->motion_model = std::bind(&PositionHeadingEKF::vehicle_model, this, _1, _2);
    predict_step->transition_matrix_fn = std::bind(&PositionHeadingEKF::transition_matrix_fn, this, _1, _2);
    predict_step->jacobian_matrix_fn = std::bind(&PositionHeadingEKF::jacobian_matrix_fn, this, _1, _2);
    states.push_back(predict_step);

    predict_step->stamp = stamp;
//    predict_step->valid_flag = true;
    predict_step->valid_individual = true;
    predict_step->valid_sequence = true;

    predict_step->motion.mean = delta_state_change;

    //process noise
    if(abs(previous_speed) < 0.3){
        predict_step->motion.noise << 0., 0.,
                                      0., 0.;
    }else if(abs(previous_speed) < 0.5){
        predict_step->motion.noise << VELOCITY_NOISE_SLOW, 0.,
                                      0., YAWRATE_NOISE_SLOW;
    }else{
        predict_step->motion.noise << VELOCITY_NOISE, 0.,
                                      0., YAWRATE_NOISE;
    }

    float delta_time = 0.01;
    predict_step->motion.noise *= delta_time;
    predict_step->motion.noise = predict_step->motion.noise.array().square();

    // connect to a linked list of predict steps
    if (last_predict) {
      last_predict->next = predict_step;
      predict_step->prev = last_predict;
    }

    last_predict = predict_step;



        //predict(delta_state_change, Q);
    predict_step->condition(prior);

    //ROS_INFO_STREAM("predict " << predict_step->posterior.mean);

    /*
       force the odom filter to use the absolute heading estimate - the odom source should be continuous
       in position but not in heading - the drifting of the baselink in the odom frame can only be
       in position. if there was an angular offset between the map->odom, this rotation
       could only be applied to the base_link directly
    */

    state_odom_only[2] = predict_step->posterior.mean[2];

    for (auto &signal_map : signal_map_state){
      signal_map(predict_step->posterior.mean, predict_step->posterior.covariance, stamp);
    }


    for (auto &signal_stat:signal_statistics){
        Eigen::Vector3d innovation;
        innovation << 0., 0., 0.;
        signal_stat(predict_step->posterior.mean, innovation, predict_step->posterior.covariance, innovation, stamp, *last_source);
    }
//    ROS_ERROR_STREAM_THROTTLE(1, "predict source: " << *last_source << stamp);

  }else{
      for (auto &signal_stat:signal_statistics){
          Eigen::Vector3d innovation;
          innovation.setZero();;
          signal_stat(state_odom_only, innovation, odom_uncertainty, innovation, stamp, *last_source);
      }
//      ROS_ERROR_STREAM_THROTTLE(1, "predict source: " << *last_source << stamp);

  }

}


void
PositionHeadingEKF::reinitialise(Eigen::Vector3d& observation, ros::Time &stamp){
    // Add directly, this is the initialisation of the filter, so it is definitely considered valid
    std::shared_ptr<UpdateStep> update_step = std::make_shared<UpdateStep>();
    states.clear();
    states.push_back(update_step);

    update_step->posterior.mean = observation;
    Eigen::Matrix3d covariance;
    covariance.setZero(); // zero uncertainty. We are certain about the initial pose
    update_step->posterior.covariance = covariance;

    update_step->observation.mean = observation;
    update_step->observation.noise = covariance;

    update_step->valid_individual = true;
    update_step->valid_sequence = false;
    update_step->stamp = stamp;

    initialised = true;
    last_source = std::make_shared<std::string>("reset_dead_reckon");
    ROS_WARN("reinitialise");
}





