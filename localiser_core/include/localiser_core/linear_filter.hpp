#ifndef linear_filter_h
#define linear_filter_h

#include "localisation_method.hpp"

#include <Eigen/Dense>
#include <Eigen/Cholesky>


#include <cmath>
#include <string>

#include <mrpt/poses/CPose2D.h>
#include <mrpt_bridge/mrpt_bridge.h>

#include <Eigen/Core>
#include <Eigen/StdVector>


//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


#include <mrpt/obs/CActionRobotMovement2D.h>

//#include <mrpt/math/CMatrixDouble33.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/poses/CPosePDFGaussian.h>
//#include <mrpt/obs/CObservationBearingRange.h>
//#include <mrpt/obs/CActionCollection.h>
//#include <mrpt/obs/CActionRobotMovement2D.h>
//#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
//#include <mrpt/gui/CDisplayWindow3D.h>
//#include <mrpt/math/lightweight_geom_data.h>
//#include <mrpt/opengl/stock_objects.h>

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

/*
using namespace mrpt;
using namespace mrpt::bayes;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::random;
using namespace std;

#define BEARING_SENSOR_NOISE_STD DEG2RAD(15.0f)
#define RANGE_SENSOR_NOISE_STD 0.3f
#define DELTA_TIME 0.1f

#define VEHICLE_INITIAL_X 4.0f
#define VEHICLE_INITIAL_Y 4.0f
#define VEHICLE_INITIAL_V 1.0f
#define VEHICLE_INITIAL_W DEG2RAD(20.0f)

#define TRANSITION_MODEL_STD_XY 0.03f
#define TRANSITION_MODEL_STD_VXY 0.20f
*/


// TODO: observe speed also ? estimate noise ?


class StateEstimate {

public:
  Eigen::Vector3d mean;
  Eigen::Matrix3d covariance;

};


class ConditionedState {
public:

  ConditionedState() : valid_flag(false) {}


  virtual bool condition(StateEstimate &prior) = 0;

  StateEstimate posterior;

  ros::Time stamp;
  bool valid_flag;

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

class RelativeMotion{

public:

  RelativeMotion() {}

  Eigen::Vector2d mean;
  Eigen::Matrix2d noise;    // Q
};


class UpdateStep: public ConditionedState {

public:
  Observation observation;

  bool condition(StateEstimate &prior);


  std::shared_ptr<UpdateStep> prev, next;
};


class PredictStep: public ConditionedState {

public:
  RelativeMotion motion;

  bool condition(StateEstimate &prior);

  std::function<Eigen::Vector3d(const Eigen::Vector3d &, const Eigen::Vector2d &)> motion_model;
  std::function<Eigen::Matrix3d(Eigen::Vector3d, Eigen::Vector2d)> transition_matrix_fn;
  std::function<Eigen::MatrixXd(Eigen::Vector3d, Eigen::Vector2d)> jacobian_matrix_fn;

  //Eigen::Vector3d motion_model(const Eigen::Vector3d &mean, const Eigen::Vector2d &input_state) {return Eigen::Vector3d();}


  std::shared_ptr<PredictStep> prev, next;
};



class LinearFilter : public LocalisationMethod {
public:

  LinearFilter();
  ~LinearFilter();

  bool initialised = false;
  float MINIMUM_INITIALISATION_SPEED = .5;

  // use hard coded uncertainty values from python code
  float VELOCITY_NOISE = 2.5; // m/s
  float YAWRATE_NOISE = 1.5 * (3.1415 / 180.0); // deg/s
  float HEADING_ERROR = 2. * (3.1415 / 180.0);// deg
  float POSITION_ERROR = 2.5; // m

  Eigen::Vector3d state_odom_only;

//  std::vector<std::unique_ptr<sf::Shape>>
//  std::deque<Observation> update_steps;
//  std::deque<RelativeMotion> prediction_steps;

  std::deque<std::shared_ptr<ConditionedState>> states;

//  Eigen::Vector3d state;
//  Eigen::Matrix3d state_var;

//  Eigen::Vector2d prior_motion;
//  Eigen::Matrix2d prior_motion_covariance;
  //ros::Time prior_stamp;

  //RelativeMotion prior_motion;

  float previous_speed = 0.;

  bool initialised_filter;
  double confidence;

  //! Perform the optimisation
  void AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Matrix2d& covariance, ros::Time stamp);
  virtual void AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time stamp) {}

  void BoundHeading(const Eigen::Vector3d &current_state, Eigen::Vector3d &observation);


  // Functions to implement for the specific problem being applied to the filter
  virtual Eigen::Vector3d vehicle_model(const Eigen::Vector3d &mean, const Eigen::Vector2d &input_state) {return Eigen::Vector3d();}
  virtual Eigen::VectorXd observation_model(const Eigen::VectorXd state, const Eigen::VectorXd observation) {return Eigen::VectorXd();}
  virtual Eigen::Matrix3d transition_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state) {return Eigen::Matrix3d();}
  virtual Eigen::MatrixXd jacobian_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state) {return Eigen::MatrixXd();}


protected:
/*
  bool update(StateEstimate &prior,
              Observation &observation,
              StateEstimate &posterior);

              //const Eigen::VectorXd &observation,
              //const Eigen::MatrixXd &observation_noise,
              //  const Eigen::MatrixXd &observation_matrix,
              //ros::Time stamp
              //


  void predict(StateEstimate &prior,
               RelativeMotion &motion,
               StateEstimate &posterior);
*/

//  void predict(const Eigen::VectorXd input_state, const Eigen::MatrixXd input_state_variance);
  //Eigen::VectorXd input_state,
  //Eigen::MatrixXd input_state_variance,


  std::shared_ptr<UpdateStep> last_update;
  std::shared_ptr<PredictStep> last_predict;

};


class PositionHeadingEKF : public LinearFilter {

public:

  PositionHeadingEKF() : LinearFilter() {}

  // Functions to implement for the specific problem being applied to the filter
  Eigen::Vector3d vehicle_model(const Eigen::Vector3d &mean, const Eigen::Vector2d &input_state);
  Eigen::VectorXd observation_model(const Eigen::VectorXd state, const Eigen::VectorXd observation);
  Eigen::Matrix3d transition_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state);
  Eigen::MatrixXd jacobian_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state);

  void AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time stamp);

  void test_predict();
  void test_update();

};



class PositionOnlyEKF : public LinearFilter {

public:

  PositionOnlyEKF() : LinearFilter() {}

  // Functions to implement for the specific problem being applied to the filter
  Eigen::Vector3d vehicle_model(const Eigen::Vector3d &mean, const Eigen::Vector2d &input_state);
  Eigen::VectorXd observation_model(const Eigen::VectorXd state, const Eigen::VectorXd observation);
  Eigen::Matrix3d transition_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state);
  Eigen::MatrixXd jacobian_matrix_fn(Eigen::Vector3d mean, Eigen::Vector2d input_state);

  void AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time stamp);


  void test_predict();
  void test_update();

};



#endif
