#ifndef graph_optimiser_h
#define graph_optimiser_h

#include "localisation_method.hpp"

#include <cmath>
#include <deque>
#include <string>

#include <mrpt/poses/CPose2D.h>
#include <mrpt_bridge/mrpt_bridge.h>

#include <gps_common/conversions.h>


#include <Eigen/Core>
#include <Eigen/StdVector>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>


//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>


#include <g2o/core/sparse_optimizer.h>

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include <mrpt/obs/CActionRobotMovement2D.h>

//#include <mrpt/math/CMatrixDouble33.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
//#include <mrpt/obs/CObservationBearingRange.h>
//#include <mrpt/obs/CActionCollection.h>
//#include <mrpt/obs/CActionRobotMovement2D.h>
//#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
//#include <mrpt/gui/CDisplayWindow3D.h>
//#include <mrpt/math/lightweight_geom_data.h>
//#include <mrpt/opengl/stock_objects.h>

using namespace mrpt::poses;
using namespace g2o;

/*!
 * \brief Optimise a graph of poses
 * Optimises a graph structure containing poses linked by relative motion, with additional edges for global
 * observations
 */

class GraphOptimiser : public LocalisationMethod {
public:
  GraphOptimiser();

  ~GraphOptimiser() {
    global_optimizer.save("finished.g2o");
    outBag.close();
  }

  rosbag::Bag outBag;
  Eigen::Matrix3d gps_information;
//  Eigen::Matrix3d odom_information;

  //Eigen::Matrix2d gps_information;
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions motion_model_options_;

  double previous_yaw_rate;
  double previous_speed;
  ros::Time previous_stamp;

  int observe_countdown;
  int predict_countdown;

  std::shared_ptr<g2o::BlockSolverX> blockSolver;

  g2o::VertexSE2* origin;
  g2o::OptimizationAlgorithmGaussNewton* algoGN;

  uint32_t current_index;
  CPosePDFGaussian current_pose;

  // The number of previous observations that are included in the optimisation process
  //  bigger number means more smoothed path
  //  smaller number means faster computation
  const uint32_t OPTIMISE_OVER_PREVIOUS_VALUES = 50;

//  const double datum_x = 332722.272927207;
//  const double datum_y = 6248431.02677212;
  double datum_x;
  double datum_y;

  std::deque<g2o::HyperGraph::EdgeSet> edge_deque;

  std::deque<std::pair<uint32_t, mrpt::poses::CPose2D>> prior_odometry;

  uint32_t previous_odom_vertex_id;

  void RunOptimiser();

  ros::Time last_odometry;

  // Store the odometry measurement times to link to delayed observations
  std::list<std::pair<uint32_t, ros::Time>> odometry_times;

  g2o::SparseOptimizer global_optimizer;

  mrpt::poses::CPose2D VehicleModel(mrpt::poses::CPose2D &previous_pose, Eigen::Vector2d &motion_delta);

  //! Perform the optimisation
  void AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Matrix2d& covariance, ros::Time stamp);
  void AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time stamp);
};


#endif
