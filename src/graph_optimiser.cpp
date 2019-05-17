﻿#include "graph_optimiser.hpp"

#include <g2o/types/slam2d/types_slam2d.h>

#include <g2o/core/factory.h>


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
  observe_countdown(0),
  origin(NULL),
  current_index(0),
  //previous_odom_vertex_id(0),
  datum_x(0.),
  datum_y(0.) {

  odom_state <<0.,0.,0.;


//  ROS_INFO_STREAM("odom_state: " << odom_state(0) <<" " << odom_state(1) <<" "<< odom_state(2));


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

  global_optimizer.setVerbose(true);
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

  // compute a fixed gps edge information matrix
  mrpt::math::CMatrixDouble33 odom_cov;
  //Eigen::Vector3d gps_cov;
  odom_cov(0,0) = 0.0025;
  odom_cov(1,1) = 0.0025;
  odom_cov(2,2) =25;
  mrpt::poses::CPose2D place_holder(0,0,0);
  mrpt::poses::CPosePDFGaussian odom(place_holder, odom_cov);
  mrpt::math::CMatrixFixedNumeric< double, 3,3 > inf_odom;
  //Eigen::Matrix3d< double, 3,3 > inf;
  odom.getInformationMatrix(inf_odom);

  odom_information = Eigen::Matrix3d::Zero();

  for(int i = 0;i<3;i++)
    for(int j = 0;j<3;j++)
      odom_information(i,j)=inf_odom(i,j);


  // compute a fixed gps edge information matrix
  mrpt::math::CMatrixDouble33 gps_cov;
  //Eigen::Vector3d gps_cov;
  gps_cov(0,0) = 25;
  gps_cov(1,1) =  25;
  gps_cov(2,2) =  1;
//  mrpt::poses::CPose2D place_holder(0,0,0);
  mrpt::poses::CPosePDFGaussian gps(place_holder, gps_cov);
  mrpt::math::CMatrixFixedNumeric< double, 3,3 > inf;
  //Eigen::Matrix3d< double, 3,3 > inf;
  gps.getInformationMatrix(inf);

  gps_information = Eigen::Matrix3d::Zero();

  for(int i = 0;i<3;i++)
    for(int j = 0;j<3;j++)
      gps_information(i,j)=inf(i,j);

}



Eigen::Vector3d
GraphOptimiser::VehicleModel(Eigen::Vector3d &previous_pose, Eigen::Vector2d &motion_delta) {
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
  double av_heading = previous_pose(2) + ((previous_yaw_rate + motion_delta(1)) / 2.);

//  ROS_INFO_STREAM("av_heading: " <<av_heading);

  previous_yaw_rate = motion_delta(1);

  double av_speed = (motion_delta(0) + previous_speed) / 2.;
  previous_speed = motion_delta(0);

//  ROS_INFO_STREAM("motion: " << motion(0) <<" "<< motion(1));

  Eigen::Vector3d odom_state(previous_pose(0) + av_speed * cos(av_heading),
      previous_pose(1) + av_speed * sin(av_heading),
      av_heading);
//  previous_pose[2] + motion_delta[1]);
   return odom_state;

}

//! Perform the optimisation
void
GraphOptimiser::AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Vector2d& covariance, ros::Time stamp){
//  ROS_INFO_STREAM("current_index: "<< current_index);


//  if(current_index > 30000)
//    return;


  // don't add graph elements while stationary (According to the odometry)
  if (motion(0) == 0){
    ROS_INFO_STREAM("stationary");
    return;}

  //! Use a fixed delta_t (100Hz) until the timing issues with the vectornav is fixed
  double delta_t = 0.01;
//  double delta_t = (stamp - previous_stamp).toSec();
//  previous_stamp = stamp;
  motion *= delta_t;
//  if(current_index < 10)
//  ROS_INFO_STREAM("last_odom_state: " << odom_state(0) <<" " << odom_state(1) <<" "<< odom_state(2));
  odom_state = this->VehicleModel(odom_state, motion);
//  if(current_index < 10)
//  ROS_INFO_STREAM("odom_state: " << odom_state(0) <<" " << odom_state(1) <<" "<< odom_state(2));



//  Only include every nth prediction step in the graph
  predict_countdown--;
  if (predict_countdown > 0) {
    return;
  }
  predict_countdown = 50;



  g2o::VertexSE2* odom_pose_vertex = new g2o::VertexSE2;
  odom_pose_vertex->setId(current_index);
  odom_pose_vertex->setEstimate(odom_state);
  global_optimizer.addVertex(odom_pose_vertex);

//  if(current_index == 1){
//    odom_pose_vertex->setFixed(true);
//  }else if(current_index == 29999){
//    odom_pose_vertex->setFixed(true);
//  }

  // Incorporate previous odometry measurement as an edge (if it exists)
  if(prior_odometry.size() > 0) {

    Eigen::Vector3d last_pose = prior_odometry.back().second;
    Eigen::Vector3d pose_difference = odom_state - last_pose;
    /*
      if (prior_odometry.size() > 0) {
        pose_difference -= prior_odometry.back().second; //previous_odom_state;
      }
    */
//    ROS_INFO_STREAM("last_pose: " << std::fixed << last_pose(0) <<" " << std::fixed << last_pose(1) <<" "<< last_pose(2));
    ROS_INFO_STREAM("motion: " << motion(0) <<" "<< motion(1));
    ROS_INFO_STREAM("pose_difference: " << std::fixed << pose_difference(0) <<" " << std::fixed << pose_difference(1) <<" "<< pose_difference(2));

    mrpt::poses::CPose2D odom_incr(pose_difference(0), pose_difference(1), pose_difference(2));

    mrpt::obs::CActionRobotMovement2D odom_move;
    mrpt::system::TTimeStamp mrpt_timestamp;
    mrpt_bridge::convert(stamp, mrpt_timestamp);
    odom_move.timestamp = mrpt_timestamp;
    odom_move.computeFromOdometry(odom_incr, motion_model_options_);

    mrpt::math::CMatrixDouble33 aux_cov;
    mrpt::poses::CPose2D mean_point;
    odom_move.poseChange->getCovarianceAndMean (aux_cov, mean_point);
    mrpt::poses::CPosePDFGaussian aux(mean_point, aux_cov);

    Eigen::Vector3d pose_odom(aux.mean.x(), aux.mean.y(), aux.mean.phi());
    ROS_INFO_STREAM("pose_odom: " << std::fixed << pose_odom(0) <<" " << std::fixed << pose_odom(1) <<" "<< pose_odom(2));


    mrpt::math::CMatrixFixedNumeric< double, 3,3 > inf;
    aux.getInformationMatrix(inf);
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
    for(int i = 0;i<3;i++)
      for(int j = 0;j<3;j++)
        information(i,j)=inf(i,j);

    g2o::EdgeSE2* odom_pose_edge = new g2o::EdgeSE2;
    //    Eigen::Vector3d odom_motion = odom_state - previous_odom_state;//(motion[0], 0., motion[1]);
    odom_pose_edge->vertices()[0] = global_optimizer.vertex(prior_odometry.back().first);//(previous_odom_vertex_id);
    odom_pose_edge->vertices()[1] = global_optimizer.vertex(odom_pose_vertex->id());
    odom_pose_edge->setMeasurement(pose_odom);
    odom_pose_edge->setInformation(odom_information);
    global_optimizer.addEdge(odom_pose_edge);
  }

  // Add an additional edge every nth node
  if(0){//prior_odometry.size() > 10) {

//    auto old_pose = prior_odometry.at(prior_odometry.size() - 10 - 1);

//    Eigen::Vector3d pose_difference = odom_state - old_pose.second;
//    /*
//      if (prior_odometry.size() > 0) {
//        pose_difference -= prior_odometry.back().second; //previous_odom_state;
//      }
//    */
//    //    ROS_INFO_STREAM("pose_difference: " << pose_difference);
//    mrpt::poses::CPose2D odom_incr(pose_difference[0], pose_difference[1], pose_difference[2]);

//    mrpt::obs::CActionRobotMovement2D odom_move;
//    mrpt::system::TTimeStamp mrpt_timestamp;
//    mrpt_bridge::convert(stamp, mrpt_timestamp);
//    odom_move.timestamp = mrpt_timestamp;
//    odom_move.computeFromOdometry(odom_incr, motion_model_options_);

//    mrpt::math::CMatrixDouble33 aux_cov;
//    mrpt::poses::CPose2D mean_point;
//    odom_move.poseChange->getCovarianceAndMean (aux_cov, mean_point);
//    mrpt::poses::CPosePDFGaussian aux(mean_point, aux_cov);

//    Eigen::Vector3d pose_odom(aux.mean.x(), aux.mean.y(), aux.mean.phi());

//    mrpt::math::CMatrixFixedNumeric< double, 3,3 > inf;
//    aux.getInformationMatrix(inf);
//    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
//    for(int i = 0;i<3;i++)
//      for(int j = 0;j<3;j++)
//        information(i,j)=inf(i,j);

//    g2o::EdgeSE2* odom_pose_edge = new g2o::EdgeSE2;
//    //    Eigen::Vector3d odom_motion = odom_state - previous_odom_state;//(motion[0], 0., motion[1]);
//    odom_pose_edge->vertices()[0] = global_optimizer.vertex(old_pose.first);//(previous_odom_vertex_id);
//    odom_pose_edge->vertices()[1] = global_optimizer.vertex(odom_pose_vertex->id());
//    odom_pose_edge->setMeasurement(pose_odom);
//    odom_pose_edge->setInformation(information);
//    global_optimizer.addEdge(odom_pose_edge);
  }

  Eigen::Vector3d pose_covariance(0,0,0);
  if (publish_odometry) {
    publish_odometry(odom_state, pose_covariance, stamp);
  }

  //  previous_prediction_stamp = stamp;
  //  previous_odom_vertex_id = current_index;

  prior_odometry.push_back(std::make_pair(current_index, odom_state));

  while (prior_odometry.size() > 25)
    prior_odometry.pop_front();

  current_index += 1;
}

void
GraphOptimiser::AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Vector3d& covariance, ros::Time stamp) {
  ROS_INFO_STREAM("ADDING ABSOLUTE POSITION");

//  if (observe_countdown > 0) {
//    observe_countdown--;
//    return;
//  }

//  observe_countdown = 1000;

  //! TODO: don't assume it fits with the latest odom

  if (datum_x == 0.) {
    datum_x = observation[0];
    datum_y = observation[1];
  }

  Eigen::Vector3d gps_measurement(observation[0] - datum_x, observation[1] - datum_y, observation[2]);
  //    Eigen::Vector2d gps_measurement(observation[0] - datum_x, observation[1] - datum_y);

  //  ROS_INFO_STREAM(current_index);
  //g2o::EdgeSE2PointXY* gps_edge = new g2o::EdgeSE2PointXY;
  g2o::EdgeSE2* gps_edge = new g2o::EdgeSE2;
  gps_edge->vertices()[0] = global_optimizer.vertex(0);
  gps_edge->vertices()[1] = global_optimizer.vertex(current_index - 1);
  gps_edge->setMeasurement(gps_measurement);
  gps_edge->setInformation(gps_information);
  global_optimizer.addEdge(gps_edge);


  //  this->RunOptimiser();

  previous_observation_stamp = stamp;
}




void
GraphOptimiser::RunOptimiser() {
  // Align feature map to utm frame
  //  global_optimizer.save("tmp.g2o");

  global_optimizer.initializeOptimization();
  global_optimizer.optimize(3);

  //global_optimizer.save("/home/stew/test.g2o");

  /*
  g2o::SparseOptimizer tmp;
  tmp.setVerbose(true);

  // create the linear solver
  //g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>> linearSolver;
  auto linearSolver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
  //linearSolver = std::move(newlinearSolver);
  // create the block solver on top of the linear solver

  //  blockSolver = std::make_shared<g2o::BlockSolverX>(std::move(linearSolver));

  auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
  //blockSolver = std::move(newblockSolver);

  // create the algorithm to carry out the optimization
  auto algoGN2 = new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
  //  OptimizationAlgorithmLevenberg* algoLM = new OptimizationAlgorithmLevenberg(std::move(blockSolver));

  tmp.setAlgorithm(algoGN2);
  tmp.load("/home/stew/test.g2o");
  tmp.initializeOptimization();
  tmp.optimize(3);

  //  global_optimizer.initializeOptimization();
  //  global_optimizer.optimize(3);
  */

}

