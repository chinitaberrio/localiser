#include "predictor.hpp"
#include <rosbag/view.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>




Predictor::Predictor() {

}

/*
void IcpMapMaker::insert_gps(double easting, double northing, double heading) {

  Eigen::Vector3d gps_measurement;
  gps_measurement << easting, northing, heading;

  EdgeSE2* gps_edge = new EdgeSE2;
  gps_edge->vertices()[0] = global_optimizer.vertex(ORIGIN_INDEX);
  gps_edge->vertices()[1] = global_optimizer.vertex(robot_pose_id);
  gps_edge->setMeasurement(gps_measurement);
  gps_edge->setInformation(gps_information);
  global_optimizer.addEdge(gps_edge);
//  gps_edges.insert(gps_edge);
}

void IcpMapMaker::insert_to_graph(mrpt_msgs::ObservationRangeBearing obs, CPose2D odom_incr) {

    CActionRobotMovement2D odom_move;
    mrpt::system::TTimeStamp mrpt_timestamp;
    mrpt_bridge::convert(obs.header.stamp, mrpt_timestamp);
    odom_move.timestamp = mrpt_timestamp;
    odom_move.computeFromOdometry(odom_incr, motion_model_options_);

    mrpt::math::CMatrixDouble33 aux_cov;
    CPose2D mean_point;
    odom_move.poseChange->getCovarianceAndMean (aux_cov, mean_point);
    mrpt::poses::CPosePDFGaussian aux(mean_point, aux_cov);

    mrpt::poses::CPosePDFGaussian current_pose_guess = current_pose + aux;

    //add robot pose vertex
    Eigen::Vector3d pose;
    pose << current_pose_guess.mean.x(), current_pose_guess.mean.y(), current_pose_guess.mean.phi();
    VertexSE2* robot = new VertexSE2;
    robot->setId(robot_pose_id);
    robot->setEstimate(pose);
    global_optimizer.addVertex(robot);
    local_odom_vertices.push_back(robot);
    current_gps_node->robot_poses.insert(robot_pose_id);
//    local_odom_id.push_back(robot_pose_id);
    if(local_odom_vertices.size() > local_buffer_size){
        local_odom_vertices.pop_front();
    }

    //add odometry edge from last robot pose vertex to this vertex
    if (robot_pose_id > ROBOT_POSE_START){

        Eigen::Vector3d pose_odom;
        pose_odom << aux.mean.x(), aux.mean.y(), aux.mean.phi();

        CMatrixFixedNumeric< double, 3,3 > inf;
        aux.getInformationMatrix(inf);
        Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
        for(int i = 0;i<3;i++)
            for(int j = 0;j<3;j++)
                information(i,j)=inf(i,j);

        EdgeSE2* odometry = new EdgeSE2;
        odometry->vertices()[0] = global_optimizer.vertex(robot_pose_id - 1);
        odometry->vertices()[1] = global_optimizer.vertex(robot_pose_id);
        odometry->setMeasurement(pose_odom);
        odometry->setInformation(information);
        global_optimizer.addEdge(odometry);
        local_odom_edges.push_back(odometry);
        if(local_odom_edges.size() > local_buffer_size-1){
            local_odom_edges.pop_front();
        }
    }

    double rangeSigmaGrowRate = 0.002;//+5.0 * z_rangeSigma.dot(range_vec.transpose());
    double rangeSigmaOffset = 0.002;
    double bearingSigmaGrowRate = 0.001;//+5.0 * z_bearingSigma.dot(range_vec.transpose());
    double bearingSigmaOffset = 0.001;//+5.0 * z_bearingSigma.dot(range_vec.transpose());

    std::set<EdgeSE2PointXY*> frame_icp_edges;   // FIFO buffer to keep 5 recent odometry vertices

    for (mrpt_msgs::SingleRangeBearingObservation srbo : obs.sensed_data){

        double range = srbo.range;
        double bearing = srbo.yaw;
        double rangeSigma = srbo.range * rangeSigmaGrowRate + rangeSigmaOffset;
        double bearingSigma = srbo.range * bearingSigmaGrowRate + bearingSigmaOffset;

        mrpt::poses::CPoint2DPDFGaussian landmarkRangeBearingFrame(mrpt::poses::CPoint2D(range, 0.));
        landmarkRangeBearingFrame.cov(0, 0) = pow(rangeSigma, 2);
        // bearingSigma*camRange: bearing is in radians, convert bearing sigma to the axis perpendicular to range,
        // which is approximately the arc length = angle * radius = bearing * range, in meters
        landmarkRangeBearingFrame.cov(1, 1) = pow(bearingSigma*range, 2);

        mrpt::poses::CPose3D rangeBearingFrameInRobot(0., 0., 0., bearing, 0., 0.);
        landmarkRangeBearingFrame.changeCoordinatesReference(rangeBearingFrameInRobot); // now landmarkRangeBearingFrame is in robot frame

        CMatrixFixedNumeric< double, 2, 2 > inf_2d;
        landmarkRangeBearingFrame.getInformationMatrix(inf_2d);
        Eigen::Matrix2d information_2d = Eigen::Matrix2d::Zero();
        for(int i = 0;i<2;i++)
            for(int j = 0;j<2;j++)
                information_2d(i,j)=inf_2d(i,j);

        EdgeSE2PointXY* landmarkObservation = new EdgeSE2PointXY;
        Eigen::Vector2d landmarkInRobot(landmarkRangeBearingFrame.mean.x(), landmarkRangeBearingFrame.mean.y());
        landmarkObservation->setMeasurement(landmarkInRobot);
        landmarkObservation->setInformation(information_2d);

        landmarkRangeBearingFrame.changeCoordinatesReference(mrpt::poses::CPose3D(current_pose_guess.mean)); // now landmarkRangeBearingFrame is in global frame        
        if(srbo.id > landmark_id || landmark_id == 0 && landmark_0_exists == false){
            Eigen::Vector2d landmarkInGlobal(landmarkRangeBearingFrame.mean.x(), landmarkRangeBearingFrame.mean.y());
            VertexPointXY* landmark = new VertexPointXY;
            landmark->setId(srbo.id);
            landmark->setEstimate(landmarkInGlobal);
//            std::cout<< "adding landmark: " << srbo.id << "to optimizer\n";

            global_optimizer.addVertex(landmark);
            current_gps_node->landmarks.insert(srbo.id);
            landmark_id = srbo.id;
//            if( landmark_id == 0 ){
            landmark_0_exists = true;
//            }
        }

        landmarkObservation->vertices()[0] = global_optimizer.vertex(robot_pose_id);
        landmarkObservation->vertices()[1] = global_optimizer.vertex(srbo.id);
        global_optimizer.addEdge(landmarkObservation);
        frame_icp_edges.insert(landmarkObservation);
    }

    local_icp_edges.push_back(frame_icp_edges);
    if(local_icp_edges.size() > local_buffer_size){
        local_icp_edges.pop_front();
    }
}
*/


