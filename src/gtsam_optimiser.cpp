#include "gtsam_optimiser.hpp"


#include <nav_msgs/Odometry.h>

#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>



GtsamOptimiser::GtsamOptimiser() :
  LocalisationMethod(),
  previous_yaw_rate(0.),
  observe_countdown(0),
  current_index(0),
  //previous_odom_vertex_id(0),
  datum_x(0.),
  datum_y(0.) {

  odom_state_eigen = Eigen::Vector3d(0,0,+1.1);
/*  //rosbag::Bag bag;
  bag.open("/home/stew/test.bag", rosbag::bagmode::Write);
  if (bag.isOpen())
    ROS_INFO_STREAM("ROS bag is open");
  else
    ROS_INFO_STREAM("ROS bag is NOT open");
*/

//  RunOptimiser();
}


void
GtsamOptimiser::RunOptimiser() {
  cout << "STarting optimisation process" << endl;
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
//  result.print("Final Result:\n");

  for (auto value: result) {
    nav_msgs::Odometry map_msg;
    Pose2 value_pose = value.value.cast<gtsam::Pose2>();
    map_msg.pose.pose.position.x = value_pose.x();
    map_msg.pose.pose.position.y = value_pose.y();
    map_msg.twist.twist.angular.z = value_pose.theta();
    //bag.write("map", ros::Time::now(), map_msg);

  }



  // 5. Calculate and print marginal covariances for all variables
//  Marginals marginals(graph, result);
//  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
//  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
//  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

  /*
  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;

  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
  // Create odometry (Between) factors between consecutive poses
  graph.emplace_shared<BetweenFactor<Pose2> >(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise);
  graph.emplace_shared<BetweenFactor<Pose2> >(2, 3, Pose2(2.0, 0.0, 0.0), odometryNoise);

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
  graph.emplace_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise);
  graph.emplace_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise);
  graph.print("\nFactor Graph:\n"); // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;
  initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
*/
}



Eigen::Vector3d
GtsamOptimiser::VehicleModel(Eigen::Vector3d &current_pose, Eigen::Vector2d &motion_delta) {

  //! av_heading is the average heading for the time period
  double av_heading = current_pose[2] + ((previous_yaw_rate + motion_delta[1]) / 2.);

  previous_yaw_rate = motion_delta[1];

  double av_speed = (motion_delta[0] + previous_speed) / 2.;
  previous_speed = motion_delta[0];

  return Eigen::Vector3d(current_pose[0] + av_speed * cos(av_heading),
      current_pose[1] + av_speed * sin(av_heading),
      current_pose[2] + motion_delta[1]);

}

//! Perform the optimisation
void
GtsamOptimiser::AddRelativeMotion(Eigen::Vector2d& motion, Eigen::Vector2d& covariance, ros::Time stamp) {
  //! Use a fixed delta_t (100Hz) until the timing issues with the vectornav is fixed
  double delta_t = 0.01;
  motion *= delta_t;
  odom_state_eigen = this->VehicleModel(odom_state_eigen, motion);


  // don't add graph elements while stationary (According to the odometry)
  if (previous_speed == 0)
    return;
/*
  // Only include every nth prediction step in the graph
  predict_countdown--;
  if (predict_countdown > 0) {
    return;
  }
  predict_countdown = 10;
*/
/*
  g2o::VertexSE2* odom_pose_vertex = new g2o::VertexSE2;
  odom_pose_vertex->setId(current_index);
  odom_pose_vertex->setEstimate(odom_state);
  global_optimizer.addVertex(odom_pose_vertex);
*/

  initialEstimate.insert(current_index, Pose2(odom_state_eigen[0], odom_state_eigen[1], odom_state_eigen[2]));

  nav_msgs::Odometry odom_msg;
  odom_msg.pose.pose.position.x = odom_state_eigen[0];
  odom_msg.pose.pose.position.y = odom_state_eigen[1];
  odom_msg.twist.twist.angular.z = odom_state_eigen[2];
  //bag.write("odom", ros::Time::now(), odom_msg);

//    ROS_INFO_STREAM("Initial " << current_index << " " << odom_state[0] << " " << odom_state[1] << " " << odom_state[2]);

  // Incorporate previous odometry measurement as an edge (if it exists)
  if(prior_odometry.size() > 0) {

    Eigen::Vector3d pose_difference = odom_state_eigen - prior_odometry.back().second;

    //motion_model_options_.gaussianModel.minStdXY = 0.05;
    //motion_model_options_.gaussianModel.minStdPHI = 0.005;

    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.005));
    graph.emplace_shared<BetweenFactor<Pose2> >(current_index, prior_odometry.back().first, Pose2(pose_difference[0], pose_difference[1], pose_difference[2]), odometryNoise);
//    ROS_INFO_STREAM("ADDING EDGE");

/*
    mrpt::poses::CPose2D odom_incr(pose_difference[0], pose_difference[1], pose_difference[2]);

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
    odom_pose_edge->setInformation(information);
    global_optimizer.addEdge(odom_pose_edge);
    */

  }

  Eigen::Vector3d pose_covariance(0,0,0);
  if (publish_odometry) {
    publish_odometry(odom_state_eigen, pose_covariance, stamp);
  }

  //  previous_prediction_stamp = stamp;
  //  previous_odom_vertex_id = current_index;

  prior_odometry.push_back(std::make_pair(current_index, odom_state_eigen));

  while (prior_odometry.size() > 250)
    prior_odometry.pop_front();

  current_index += 1;

}

void
GtsamOptimiser::AddAbsolutePosition(Eigen::Vector3d& observation, Eigen::Vector3d& covariance, ros::Time stamp) {

  if(prior_odometry.size() == 0) {
    return;
  }

  /*
  if (observe_countdown > 0) {
    observe_countdown--;
    return;
  }

  observe_countdown = 1000;
*/
  //! TODO: don't assume it fits with the latest odom

  if (datum_x == 0.) {
    datum_x = observation[0];
    datum_y = observation[1];
  }

  Eigen::Vector3d gps_measurement(observation[0] - datum_x, observation[1] - datum_y, observation[2]);


  nav_msgs::Odometry map_msg;
  map_msg.pose.pose.position.x = gps_measurement[0];
  map_msg.pose.pose.position.y = gps_measurement[1];
  //bag.write("gnss", ros::Time::now(), map_msg);


  //    Eigen::Vector2d gps_measurement(observation[0] - datum_x, observation[1] - datum_y);
/*
  //  ROS_INFO_STREAM(current_index);
  //g2o::EdgeSE2PointXY* gps_edge = new g2o::EdgeSE2PointXY;
  g2o::EdgeSE2* gps_edge = new g2o::EdgeSE2;
  gps_edge->vertices()[0] = global_optimizer.vertex(0);
  gps_edge->vertices()[1] = global_optimizer.vertex(current_index - 1);
  gps_edge->setMeasurement(gps_measurement);
  gps_edge->setInformation(gps_information);
  global_optimizer.addEdge(gps_edge);

  //  this->RunOptimiser();
*/

  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas(Vector2(25, 25)); // 10cm std on x,y
  graph.emplace_shared<UnaryFactor>(prior_odometry.back().first, gps_measurement[0], gps_measurement[1], unaryNoise);
 // ROS_INFO_STREAM("ADDING ABSOLUTE POSITION");


  previous_observation_stamp = stamp;
}


/*

void
GraphOptimiser::RunOptimiser() {
  // Align feature map to utm frame
  //  global_optimizer.save("tmp.g2o");

  global_optimizer.initializeOptimization();
  global_optimizer.optimize(3);

  //global_optimizer.save("/home/stew/test.g2o");


}

*/


