#include "observation_model.h"


  void ICPObservation::receive_ICP(double x, double y, double heading, ros::Time stamp) {
    if (signal_update) {


      // covariance [4] is y-orientation

      // covariance is the diagonal of term 0 (x) term 1 (y) and term 5 (heading)
      Eigen::Matrix3d observation_covariance;
      if(false){
          // TODO: we haven't developed ICP with covariance currently, ICP msg doesn't provide cov, cov is currently all 0.
//          observation_covariance << msg->pose.covariance[0 + 0 * 6], 0., 0.,
//              0., msg->pose.covariance[1 + 1 * 6], 0.,
//              0., 0., msg->pose.covariance[5 + 5 * 6];
      }else{
          // assign fixed covariance
          observation_covariance << POSITION_COVARIANCE, 0., 0.,
              0., POSITION_COVARIANCE, 0.,
              0., 0., HEADING_COVARIANCE;
      }

      Eigen::Vector3d observation_vector;
      observation_vector << x, y, heading;

      std::string source = "map";
      // covariance [0] is x-twist-linear
      signal_update(observation_vector,
                    observation_covariance,
                    stamp,
                    source);
     }
  }




// PointcloudObservation do not need signal_update function pointer
// its output is ICP msgs, signals go thru SourceInterface receive_odom_SE2_msg() method
  PointcloudObservation::PointcloudObservation(std::shared_ptr<BagSource> &bag_source) {

      features_pipeline = std::make_shared<PointCloudFeaturesPipeline>();
      icp_pipeline = std::make_shared<ICPMatcherPipeline>();

      features_pipeline->publish_poles_corners = std::bind(&ICPMatcherPipeline::receive_message, &(*icp_pipeline),
                                                         std::placeholders::_1, std::placeholders::_2);

      icp_pipeline->publish_pose = std::bind(&BagSource::receive_odom_SE2_msg, &(*bag_source), std::placeholders::_1);
  }

  void PointcloudObservation::receive_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg) {
        features_pipeline->receive_message(pc_msg);
  }







  void GNSSObservation::receive_gps(double lat, double lon, int fix_status, double msg_cov, ros::Time stamp) {
//      ROS_WARN("reached GNSSObservation::receive_gps");

      if (signal_update) {
          double east, north;
          char zone[8];

          gps_common::LLtoUTM(lat, lon, north, east, zone);

          // if this msg is the first received, or the speed is less than speed threshold, or fix_status is no_fix,
          // don't send update
          if (previous_east == 0. && previous_north == 0. ) {
            previous_east = east;
            previous_north = north;
            return;
          }

//          ROS_WARN_STREAM("reached GNSSObservation::receive_gps " << east << " " << north << " " << heading );

          Eigen::Matrix3d covariance;

          if (msg_cov < RTK_COVARIANCE_THRESHOLD && fix_status == 2) {
            covariance << POSITION_COVARIANCE_RTK, 0., 0.,
                0., POSITION_COVARIANCE_RTK, 0.,
                0., 0., HEADING_COVARIANCE_RTK;
//          }else if(msg_cov <= POSITION_COVARIANCE_GNSS){
//            covariance << POSITION_COVARIANCE_GNSS, 0., 0.,
//                0., POSITION_COVARIANCE_GNSS, 0.,
//                0., 0., HEADING_COVARIANCE_GNSS;
          }else{
              covariance << msg_cov, 0., 0.,
                  0., msg_cov, 0.,
                  0., 0., HEADING_COVARIANCE_GNSS;
          }


//          ROS_WARN_STREAM("dist squared: " << std::hypot(north-previous_north, east - previous_east) << " SPEED_THRESHOLD_SQUARED " << SPEED_THRESHOLD_SQUARED);

          Eigen::Vector3d observation;

          if(fix_status == -1
             || std::hypot(north-previous_north, east - previous_east) < SPEED_THRESHOLD_SQUARED){
              observation << east, north, std::numeric_limits<double>::quiet_NaN();
          }else{
              double heading = atan2(north-previous_north, east - previous_east);
              observation << east, north, heading;
          }

          std::string source = "gnss";
          // send update to localiser method
          signal_update(observation, covariance, stamp, source);

          previous_east = east;
          previous_north = north;
      }

  }


