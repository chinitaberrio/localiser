#include "graph_optimiser.hpp"

#include <g2o/types/slam2d/types_slam2d.h>

#include <g2o/core/factory.h>

#include <chrono>

#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>



GraphOptimiser::GraphOptimiser() :
  LocalisationMethod(),
  predict_countdown(1),
  previous_yaw_rate(0.),
  previous_speed(0.),
  observe_countdown(1),
  origin(NULL),
  current_index(0),
  //previous_odom_vertex_id(0),
  datum_x(0.),
  datum_y(0.) {

//  odom_state <<0.,0.,0.;


//  ROS_INFO_STREAM("odom_state: " << odom_state(0) <<" " << odom_state(1) <<" "<< odom_state(2));

  edge_deque.push_back(g2o::HyperGraph::EdgeSet());


  Eigen::Vector3d pose(0,0,0);
  origin = new g2o::VertexSE2;
  //origin = std::make_shared<g2o::VertexSE2>();
  origin->setId(current_index++);
  origin->setEstimate(pose);
  origin->setFixed(true);
  global_optimizer.addVertex(origin);

  // create the linear solver
  //g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>> linearSolver;
  auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
  //linearSolver = std::move(newlinearSolver);
  // create the block solver on top of the linear solver

  //  blockSolver = std::make_shared<g2o::BlockSolverX>(std::move(linearSolver));

  auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
  //blockSolver = std::move(newblockSolver);

  // create the algorithm to carry out the optimization
  algoGN = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
  //  OptimizationAlgorithmLevenberg* algoLM = new OptimizationAlgorithmLevenberg(std::move(blockSolver));

  global_optimizer.setVerbose(false);
  global_optimizer.setAlgorithm(algoGN);



  // Set the options
  //motion_model_options_.modelSelection = CActionRobotMovement2D::mmThrun;
  motion_model_options_.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
  motion_model_options_.gaussianModel.a1 = 0.000000034;
  motion_model_options_.gaussianModel.a2 = 0.000000057;
  motion_model_options_.gaussianModel.a3 = 0.000000014;
  motion_model_options_.gaussianModel.a4 = 0.000000097;
  motion_model_options_.gaussianModel.minStdXY = 0.05;
  motion_model_options_.gaussianModel.minStdPHI = 0.005;

//  // compute a fixed gps edge information matrix
//  mrpt::math::CMatrixDouble33 odom_cov;
//  //Eigen::Vector3d gps_cov;
//  odom_cov(0,0) = 0.0025;
//  odom_cov(1,1) = 0.0025;
//  odom_cov(2,2) = 0.0001;
//  mrpt::poses::CPose2D place_holder(0,0,0);
//  mrpt::poses::CPosePDFGaussian odom(place_holder, odom_cov);
//  mrpt::math::CMatrixFixedNumeric< double, 3,3 > inf_odom;
//  //Eigen::Matrix3d< double, 3,3 > inf;
//  odom.getInformationMatrix(inf_odom);

//  odom_information = Eigen::Matrix3d::Zero();

//  for(int i = 0;i<3;i++)
//    for(int j = 0;j<3;j++)
//      odom_information(i,j)=inf_odom(i,j);


  // compute a fixed gps edge information matrix
  mrpt::math::CMatrixDouble33 gps_cov;
  //Eigen::Vector3d gps_cov;
  gps_cov(0,0) = 25;
  gps_cov(1,1) =  25;
  gps_cov(2,2) =  9;
  mrpt::poses::CPose2D place_holder(0,0,0);
  mrpt::poses::CPosePDFGaussian gps(place_holder, gps_cov);
  mrpt::math::CMatrixFixedNumeric< double, 3,3 > inf;
  //Eigen::Matrix3d< double, 3,3 > inf;
  gps.getInformationMatrix(inf);

  gps_information = Eigen::Matrix3d::Zero();

  for(int i = 0;i<3;i++)
    for(int j = 0;j<3;j++)
      gps_information(i,j)=inf(i,j);

}



mrpt::poses::CPose2D
GraphOptimiser::VehicleModel(mrpt::poses::CPose2D &previous_pose, Eigen::Vector2d &motion_delta) {
  /*! Vehicle transition model.

    \brief Transition function for the vehicle model

    Args:
        previous_pose (Euler::Vector3d): state [x (meters),
                                               y (meters),
                                               heading (radians)]

        motion (Eigen::Vector2d): [delta_position (meters),
                                   delta_heading (radians)]
    */

  //! av_heading is the average heading for the time period
  double av_heading = previous_pose.phi() + ((previous_yaw_rate + motion_delta(1)) / 2.);
//if(current_index < 3)
//  ROS_INFO_STREAM("av_heading: " <<av_heading);

  previous_yaw_rate = motion_delta(1);

  double av_speed = (motion_delta(0) + previous_speed) / 2.;
  previous_speed = motion_delta(0);
//if(current_index < 3)
//  ROS_INFO_STREAM("av_speed: " << av_speed);

  mrpt::poses::CPose2D odom_state_after(previous_pose.x() + av_speed * cos(av_heading),
      previous_pose.y() + av_speed * sin(av_heading),
      av_heading);
//  previous_pose[2] + motion_delta[1]);
//  if(current_index < 3)
//  ROS_INFO_STREAM("odom_state_after: " << odom_state_after(0) <<" " << odom_state_after(1) <<" "<< odom_state_after(2));

//  odom_state_after.normalizePhi();
  return odom_state_after;

}

//! Perform the optimisation
void
GraphOptimiser::AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Matrix2d& covariance, ros::Time stamp){

  // don't add graph elements while stationary (According to the odometry)
  if (motion(0) == 0){
    ROS_INFO_STREAM_THROTTLE(1., "Rejecting stationary measurements");
    return;
  }

  //how to do this
  // Fix the heading of the odometry source - the odom transform is continuous in position but not in heading
  //odom_state.phi(map_pose[2]);



  g2o::VertexSE2* odom_pose_vertex = new g2o::VertexSE2;
  odom_pose_vertex->setId(current_index);

  //! Use a fixed delta_t (100Hz) until the timing issues with the vectornav is fixed
  // motion deals with rates (velocity, yaw rate) and multiplying by delta t integrates this signal
  double delta_t = 0.01;
  motion *= delta_t;

  odom_state = this->VehicleModel(odom_state, motion);
  odom_state_eigen = Eigen::Vector3d(odom_state.x(),odom_state.y(),odom_state.phi());

  if (global_optimizer.vertices().size() > 0) {
    std::vector<double> data(3);
    global_optimizer.vertex(current_index-1)->getEstimateData(&data[0]);
    mrpt::poses::CPose2D map_pose(data[0], data[1], data[2]);
    map_pose = this->VehicleModel(map_pose, motion);
    Eigen::Vector3d map_pose_eigen(map_pose.x(),map_pose.y(),map_pose.phi());
    odom_pose_vertex->setEstimate(map_pose_eigen);
  }
  else {
    odom_pose_vertex->setEstimate(odom_state_eigen);
  }

  global_optimizer.addVertex(odom_pose_vertex);

  // store the times of the odometry measurements for cross referencing with the absolute measurements
  odometry_times.push_front(std::make_pair(current_index, stamp));

  // Generate an edge based on the previous odometry measurement (if it exists)
  if(prior_odometry.size() > 0) {
    mrpt::poses::CPose2D last_pose = prior_odometry.back().second;
    mrpt::poses::CPose2D odom_incr = odom_state - last_pose;

    // don't need to do this anymore ?
    // odom_incr.normalizePhi();

    // calculate vehicle movement between poses
    mrpt::obs::CActionRobotMovement2D odom_move;
    mrpt::system::TTimeStamp mrpt_timestamp;
    mrpt_bridge::convert(stamp, mrpt_timestamp);
    odom_move.timestamp = mrpt_timestamp;
    odom_move.computeFromOdometry(odom_incr, motion_model_options_);

    // calculate covariance
    mrpt::math::CMatrixDouble33 aux_cov;
    mrpt::poses::CPose2D mean_point;
    odom_move.poseChange->getCovarianceAndMean (aux_cov, mean_point);
    mrpt::poses::CPosePDFGaussian aux(mean_point, aux_cov);

    Eigen::Vector3d pose_odom(aux.mean.x(), aux.mean.y(), aux.mean.phi());

    mrpt::math::CMatrixFixedNumeric< double, 3,3 > inf;
    aux.getInformationMatrix(inf);
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    for(int i = 0;i<3;i++)
      for(int j = 0;j<3;j++)
        information(i,j)=inf(i,j);

    // generate the edge between odometry poses
    g2o::EdgeSE2* odom_pose_edge = new g2o::EdgeSE2;
    odom_pose_edge->vertices()[0] = global_optimizer.vertex(prior_odometry.back().first);//(previous_odom_vertex_id);
    odom_pose_edge->vertices()[1] = global_optimizer.vertex(odom_pose_vertex->id());
    odom_pose_edge->setMeasurement(pose_odom);
    odom_pose_edge->setInformation(information);
    global_optimizer.addEdge(odom_pose_edge);

    // Add to the set of edges that belong to the most recent observation
    (edge_deque.back()).insert(odom_pose_edge);
  }

  // store the previous 25 odom states with the associated graph node index
  prior_odometry.push_back(std::make_pair(current_index, odom_state));

  while (prior_odometry.size() > 25)
    prior_odometry.pop_front();

  // extract the most recent position from the graph - after the latest optimisation this includes
  // odometry increments in the time since the optimisation
  std::vector<double> data(3);
  odom_pose_vertex->getEstimateData(&data[0]);
  Eigen::Vector3d map_pose(data[0] + datum_x, data[1] + datum_y, data[2]);

  // TODO: correct the uncertainty measurements
  Eigen::Matrix3d map_uncertainty;
  Eigen::Matrix3d odom_uncertainty;

  map_uncertainty << 0., 0., 0.,
                     0., 0., 0.,
                     0., 0., 0.;

  odom_uncertainty << 0., 0., 0.,
                      0., 0., 0.,
                      0., 0., 0.;

  if (publish_odometry)
    publish_odometry(odom_state_eigen, odom_uncertainty, stamp);

  if (publish_map)
    publish_map(map_pose, map_uncertainty, odom_state_eigen, stamp);

  current_index += 1;
}

void
GraphOptimiser::AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Matrix3d& covariance, ros::Time stamp) {

  // find the nearest relative odometry index corresponding to this observation
  uint32_t nearest_odometry_vertex = 0;

  for (auto &odometry_time: odometry_times) {
    if (odometry_time.second < stamp) {
      nearest_odometry_vertex = odometry_time.first;
      if (nearest_odometry_vertex < 0)
        nearest_odometry_vertex = 0;

      break;
    }
  }

  // TODO: take the datum from the ros param server
  // set the datum to the first absolute measurement
  if (datum_x == 0.) {
    datum_x = observation[0];
    datum_y = observation[1];
  }

  Eigen::Vector3d position_prior_estimate(observation[0] - datum_x, observation[1] - datum_y, observation[2]);

  // push the relative motion associated with this observation into the list of included edges
  edge_deque.push_back(g2o::HyperGraph::EdgeSet());

  // create an edge for the absolute position (to the 'origin' vertex)
  g2o::EdgeSE2* absolute_edge = new g2o::EdgeSE2;
  absolute_edge->vertices()[0] = global_optimizer.vertex(0);
  absolute_edge->vertices()[1] = global_optimizer.vertex(nearest_odometry_vertex);

  absolute_edge->setMeasurement(position_prior_estimate);
  absolute_edge->setInformation(gps_information);
  global_optimizer.addEdge(absolute_edge);

  edge_deque.back().insert(absolute_edge);

  while (edge_deque.size() > OPTIMISE_OVER_PREVIOUS_VALUES)
    edge_deque.pop_front();

  // no point to optimise without at least two observations
  if (run_optimiser_each_observation && edge_deque.size() > 2)
    this->RunOptimiser();

  // extract the best position estimate after optimisation
  std::vector<double> position_posterior_estimate(3);
  global_optimizer.vertex(nearest_odometry_vertex)->getEstimateData(&position_posterior_estimate[0]);

  double innovation = sqrt(pow(position_prior_estimate[0] - position_posterior_estimate[0], 2) + pow(position_prior_estimate[1] - position_posterior_estimate[1], 2));
  ROS_INFO_STREAM_THROTTLE(0.5, innovation);

  // set the time recorded for the latest observation
  previous_observation_stamp = stamp;
}



void
GraphOptimiser::RunOptimiser(bool global) {
  // Align feature map to utm frame
  auto start = std::chrono::steady_clock::now();

  g2o::HyperGraph::EdgeSet edge_set;

  if (!global) {
    for (auto edge_subset: edge_deque) {
      edge_set.insert(edge_subset.begin(), edge_subset.end());
    }
  }

  if (global_optimizer.vertices().size() < 2) {
    ROS_INFO_STREAM_THROTTLE(1,
        "Not running optimiser with only " << global_optimizer.vertices().size()
                                  << " vertices");
  }
  else {
    if (global) {
      global_optimizer.initializeOptimization();
    }
    else {
      global_optimizer.initializeOptimization(edge_set);
    }

    global_optimizer.optimize(3);
  }

  auto end = std::chrono::steady_clock::now();
}

