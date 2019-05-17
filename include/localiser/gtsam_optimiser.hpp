#ifndef gtsam_optimiser_h
#define gtsam_optimiser_h

#include "localisation_method.hpp"


#include <rosbag/bag.h>

#include <cmath>
#include <deque>
#include <string>

#include <gps_common/conversions.h>


#include <Eigen/Core>
#include <Eigen/StdVector>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <rosbag/bag.h>


//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>




// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>

// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// As in OdometryExample.cpp, we use a BetweenFactor to model odometry measurements.
#include <gtsam/slam/BetweenFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace gtsam;

// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>

class UnaryFactor: public NoiseModelFactor1<Pose2> {

  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  double mx_, my_;

public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model):
    NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

  virtual ~UnaryFactor() {}

  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.
  Vector evaluateError(const Pose2& q, boost::optional<Matrix&> H = boost::none) const
  {
    // The measurement function for a GPS-like measurement is simple:
    // error_x = pose.x - measurement.x
    // error_y = pose.y - measurement.y
    // Consequently, the Jacobians are:
    // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
    // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
    if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0).finished();
    return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.

}; // UnaryFactor



/*!
 * \brief Optimise a graph of poses
 * Optimises a graph structure containing poses linked by relative motion, with additional edges for global
 * observations
 */

class GtsamOptimiser : public LocalisationMethod {
public:
  GtsamOptimiser();

  ~GtsamOptimiser() {RunOptimiser();
                     bag.close();}


  rosbag::Bag bag;

  Eigen::Matrix3d gps_information;

  double previous_yaw_rate;
  double previous_speed;

  int observe_countdown;
  int predict_countdown;


  uint32_t current_index;

  double datum_x;
  double datum_y;

  std::deque<std::pair<uint32_t, Eigen::Vector3d>> prior_odometry;
  NonlinearFactorGraph graph;
  Values initialEstimate;

  void RunOptimiser();

  Eigen::Vector3d VehicleModel(Eigen::Vector3d &current_pose, Eigen::Vector2d &motion_delta);


  //! Perform the optimisation
  void AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Vector2d& covariance, ros::Time stamp);

  void AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Vector3d& covariance, ros::Time stamp);
};


#endif
