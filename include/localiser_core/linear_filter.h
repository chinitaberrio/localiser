#ifndef linear_filter_h
#define linear_filter_h

#include "localisation_method.h"

//#include <Eigen/Dense>
//#include <Eigen/Cholesky>


#include <cmath>
#include <string>

#include <mrpt/poses/CPose2D.h>

//#include <Eigen/Core>
//#include <Eigen/StdVector>


//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/math/CMatrixFixed.h>
//#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/poses/CPosePDFGaussian.h>


/*!
 * \brief Optimise a graph of poses
 * Optimises a graph structure containing poses linked by relative motion, with additional edges for global
 * observations
 */


// ------------------------------------------------------
//  Refer to the description in the wiki:
//  https://www.mrpt.org/Kalman_Filters
// ------------------------------------------------------

#include <mrpt/bayes/CKalmanFilterCapable.h>
#include <mrpt/bayes/CParticleFilterData.h>

#include <mrpt/gui/CDisplayWindowPlots.h>
#include <mrpt/math/distributions.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/random.h>
#include <mrpt/system/os.h>
#include <iostream>


// TODO: observe speed also ? estimate noise ?


class StateEstimate {

public:
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance;

};


class ConditionedState {
public:

  ConditionedState() : valid_individual(true), valid_sequence(true) {}


  virtual bool condition(StateEstimate &prior) = 0;

  StateEstimate posterior;

  ros::Time stamp;
  bool valid_individual;
  bool valid_sequence;

};

class RelativeMotion{

public:

  RelativeMotion() {}

  Eigen::Vector2d mean;
  Eigen::Matrix2d noise;    // Q
};


class PredictStep: public ConditionedState {

public:
  RelativeMotion motion;

  bool condition(StateEstimate &prior);

  std::function<Eigen::Vector3d(const Eigen::Vector3d &, const Eigen::Vector2d &)> motion_model;
  std::function<Eigen::Matrix3d(Eigen::Vector3d, Eigen::Vector2d)> transition_matrix_fn;
  std::function<Eigen::MatrixXd(Eigen::Vector3d, Eigen::Vector2d)> jacobian_matrix_fn;

  //Eigen::Vector3d motion_model(const Eigen::Vector3d &mean, const Eigen::Vector2d &input_state) {return Eigen::Vector3d();}


  std::shared_ptr<PredictStep> prev, next;    ///////////////////////
};




class Observation{

public:

  Observation() {
    //observation_matrix << 1., 0., 0.,
    //                      0., 1., 0.;
  }

  Eigen::Vector3d mean;     // z
  Eigen::MatrixXd noise;    // R

  Eigen::MatrixXd observation_matrix;   // H

};

class UpdateStep: public ConditionedState {

public:
  Observation observation;

  bool condition(StateEstimate &prior);


  bool CalculateConsensus(float consensus_window, std::deque<std::shared_ptr<ConditionedState>> &states);

  float speed_discrepancy;

  Eigen::Vector3d v_3d;
  Eigen::Vector3d chi_95_3d;
  Eigen::Matrix3d covariance_3d;

  double confidence = 1.;
  double next_confidence = 1.;


  std::shared_ptr<UpdateStep> prev, next;  ///////////////////
};


class LinearFilter : public LocalisationMethod {
public:

  LinearFilter();
//  ~LinearFilter();

  bool initialised = false;             ////////////////
  float MINIMUM_INITIALISATION_SPEED = .5;

  // use hard coded uncertainty values from python code
  float VELOCITY_NOISE = 3.5; // m/s
  float YAWRATE_NOISE = 5. * (3.1415 / 180.0); // deg/s
  float VELOCITY_NOISE_SLOW = 0.5; // m/s
  float YAWRATE_NOISE_SLOW = 0.7 * (3.1415 / 180.0); // deg/s
  float HEADING_ERROR = 2. * (3.1415 / 180.0);// deg
  float POSITION_ERROR = 2.5; // m

  Eigen::Vector3d state_odom_only;
  std::deque<std::shared_ptr<ConditionedState>> states;       //////////////////


  float previous_speed = 0.;          ///////////////

//  double confidence;      ////////////

  //! Perform the optimisation
//  void AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Matrix2d& covariance, ros::Time stamp);
  virtual void AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time stamp, std::string &source) {
      ROS_ERROR("calling unimplemented AddAbsolutePosition()");
  }

  void BoundHeading(const Eigen::Vector3d &current_state, Eigen::Vector3d &observation);

protected:
  std::shared_ptr<UpdateStep> last_update;     ////////////////////
  std::shared_ptr<PredictStep> last_predict;    //////////////////////

};


class PositionHeadingEKF : public LinearFilter {

public:

  PositionHeadingEKF() : LinearFilter() {}

  // Functions to implement for the specific problem being applied to the filter
  Eigen::Vector3d vehicle_model(const Eigen::Vector3d &mean, const Eigen::Vector2d &input_state);
  Eigen::Matrix3d transition_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state);
  Eigen::MatrixXd jacobian_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state);

  void AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time &stamp, std::string &source);
  void AddRelativeMotion(Eigen::Vector3d& increment, Eigen::Matrix3d& increment_cov, ros::Time &stamp);
  void reinitialise(Eigen::Vector3d& observation, ros::Time &stamp);

  void test_predict();
  void test_update();

};



#endif
