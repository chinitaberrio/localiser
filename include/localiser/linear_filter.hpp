#ifndef linear_filter_h
#define linear_filter_h

#include "localisation_method.hpp"


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

#define NUM_PARTICLES 2000


// observe speed also ? estimate noise ?

class LinearFilter : public LocalisationMethod,  public mrpt::bayes::CKalmanFilterCapable<3 /* x y yaw*/, 2 /* x y */, 0, 3 /* Atime, speed, yaw rate */> {
public:
//  LinearFilter();


  Eigen::Matrix3d gps_information;
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions motion_model_options_;


  void getState(KFVector& xkk, KFMatrix& pkk)
  {
      xkk = m_xkk;
      pkk = m_pkk;
  }

 protected:
  float m_obsBearing, m_obsRange;
  float m_deltaTime;

  /*
  void RunOptimiser();

  g2o::SparseOptimizer global_optimizer;

  Eigen::Vector3d VehicleModel(Eigen::Vector3d &current_pose, Eigen::Vector2d &motion_delta);

  //! Perform the optimisation
  void AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Vector2d& covariance, ros::Time stamp);

  void AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Vector3d& covariance, ros::Time stamp);
*/





  LinearFilter()
  {
      // KF_options.method = kfEKFNaive;
      KF_options.method = kfEKFAlaDavison;

      // INIT KF STATE
      m_xkk.resize(4);  // State: (x,y,heading,v,w)
      m_xkk[0] = VEHICLE_INITIAL_X;
      m_xkk[1] = VEHICLE_INITIAL_Y;
      m_xkk[2] = -VEHICLE_INITIAL_V;
//      m_xkk[3] = 0;

      // Initial cov:  Large uncertainty
      m_pkk.setSize(3, 3);
      m_pkk.unit();
      m_pkk(0, 0) = m_pkk(1, 1) = square(5.0f);
      m_pkk(2, 2) = square(1.0f);
  }

  ~LinearFilter() {}

  void doProcess(
      double DeltaTime, double observationRange, double observationBearing)
  {
      m_deltaTime = (float)DeltaTime;
      m_obsBearing = (float)observationBearing;
      m_obsRange = (float)observationRange;

      runOneKalmanIteration();
  }

  /** Must return the action vector u.
   * \param out_u The action vector which will be passed to OnTransitionModel
   */
  void OnGetAction(KFArray_ACT& u) const { u[0] = m_deltaTime; }
  /** Implements the transition model \f$ \hat{x}_{k|k-1} = f( \hat{x}_{k-1|k-1},
   * u_k ) \f$
   * \param in_u The vector returned by OnGetAction.
   * \param inout_x At input has \f$ \hat{x}_{k-1|k-1} \f$, at output must have
   * \f$ \hat{x}_{k|k-1} \f$.
   * \param out_skip Set this to true if for some reason you want to skip the
   * prediction step (to do not modify either the vector or the covariance).
   * Default:false
   */
  void OnTransitionModel(
      const KFArray_ACT& in_u, KFArray_VEH& inout_x,
      bool& out_skipPrediction) const
  {
      // in_u[0] : Delta time
      // in_out_x: [0]:x  [1]:y  [3]:heading
      inout_x[0] += in_u[0] * inout_x[2];
      inout_x[1] += in_u[0] * inout_x[3];
  }

  /** Implements the transition Jacobian \f$ \frac{\partial f}{\partial x} \f$
   * \param out_F Must return the Jacobian.
   *  The returned matrix must be \f$N \times N\f$ with N being either the size
   * of the whole state vector or get_vehicle_size().
   */
  void OnTransitionJacobian(KFMatrix_VxV& F) const
  {
      F.unit();

      F(0, 2) = m_deltaTime;
      F(1, 3) = m_deltaTime;
  }

  /** Implements the transition noise covariance \f$ Q_k \f$
   * \param out_Q Must return the covariance matrix.
   *  The returned matrix must be of the same size than the jacobian from
   * OnTransitionJacobian
   */
  void OnTransitionNoise(KFMatrix_VxV& Q) const
  {
      Q(0, 0) = Q(1, 1) = square(TRANSITION_MODEL_STD_XY);
      Q(2, 2) = Q(3, 3) = square(TRANSITION_MODEL_STD_VXY);
  }

  /** Return the observation NOISE covariance matrix, that is, the model of the
   * Gaussian additive noise of the sensor.
   * \param out_R The noise covariance matrix. It might be non diagonal, but it'll
   * usually be.
   * \note Upon call, it can be assumed that the previous contents of out_R are
   * all zeros.
   */
  void OnGetObservationNoise(KFMatrix_OxO& R) const
  {
      R(0, 0) = square(BEARING_SENSOR_NOISE_STD);
      R(1, 1) = square(RANGE_SENSOR_NOISE_STD);
  }

  void OnGetObservationsAndDataAssociation(
      vector_KFArray_OBS& out_z, std::vector<int>& out_data_association,
      const vector_KFArray_OBS& in_all_predictions, const KFMatrix& in_S,
      const std::vector<size_t>& in_lm_indices_in_S, const KFMatrix_OxO& in_R)
  {
      out_z.resize(1);
      out_z[0][0] = m_obsBearing;
      out_z[0][1] = m_obsRange;

      out_data_association.clear();  // Not used
  }

  /** Implements the observation prediction \f$ h_i(x) \f$.
   * \param idx_landmark_to_predict The indices of the landmarks in the map whose
   * predictions are expected as output. For non SLAM-like problems, this input
   * value is undefined and the application should just generate one observation
   * for the given problem.
   * \param out_predictions The predicted observations.
   */
  void OnObservationModel(
      const std::vector<size_t>& idx_landmarks_to_predict,
      vector_KFArray_OBS& out_predictions) const
  {
      // predicted bearing:
      kftype x = m_xkk[0];
      kftype y = m_xkk[1];

      kftype h_bear = atan2(y, x);
      kftype h_range = sqrt(square(x) + square(y));

      // idx_landmarks_to_predict is ignored in NON-SLAM problems
      out_predictions.resize(1);
      out_predictions[0][0] = h_bear;
      out_predictions[0][1] = h_range;
  }

  /** Implements the observation Jacobians \f$ \frac{\partial h_i}{\partial x} \f$
   * and (when applicable) \f$ \frac{\partial h_i}{\partial y_i} \f$.
   * \param idx_landmark_to_predict The index of the landmark in the map whose
   * prediction is expected as output. For non SLAM-like problems, this will be
   * zero and the expected output is for the whole state vector.
   * \param Hx  The output Jacobian \f$ \frac{\partial h_i}{\partial x} \f$.
   * \param Hy  The output Jacobian \f$ \frac{\partial h_i}{\partial y_i} \f$.
   */
  void OnObservationJacobians(
      const size_t& idx_landmark_to_predict, KFMatrix_OxV& Hx,
      KFMatrix_OxF& Hy) const
  {
      // predicted bearing:
      kftype x = m_xkk[0];
      kftype y = m_xkk[1];

      Hx.zeros();
      Hx(0, 0) = -y / (square(x) + square(y));
      Hx(0, 1) = 1 / (x * (1 + square(y / x)));

      Hx(1, 0) = x / sqrt(square(x) + square(y));
      Hx(1, 1) = y / sqrt(square(x) + square(y));

      // Hy: Not used
  }

  /** Computes A=A-B, which may need to be re-implemented depending on the
   * topology of the individual scalar components (eg, angles).
   */
  void OnSubstractObservationVectors(
      KFArray_OBS& A, const KFArray_OBS& B) const
  {
      A -= B;
      math::wrapToPiInPlace(A[0]);  // The angular component
  }











};


#endif
