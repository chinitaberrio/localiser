#ifndef observer_h
#define observer_h

#include <cmath>
#include <string>

#include <gps_common/conversions.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>


//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>


/*!
 * \brief The base class for providing observations
 * Class type that gives an observation of the current robot pose in the
 * global frame of reference
 */


class ObservationModel {
public:
  ObservationModel();

  std::function<void(Eigen::Vector3d&, Eigen::Matrix3d&, std::string&, ros::Time)> signal_update;



  /// define parameters such as min/max reliable measurements, lost parameters, etc
};


#include <nav_msgs/Odometry.h>

class ICPObservation : public ObservationModel {
public:
  ICPObservation() {}

  void receive_ICP(double x, double y, double heading, ros::Time stamp) {
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
      observation_vector << x, y, yaw;

      std::string source = "map";
      // covariance [0] is x-twist-linear
      signal_update(observation_vector,
                    observation_covariance,
                    source,
                    stamp);
     }
  }

private:

  const double POSITION_COVARIANCE = pow(0.3, 2); // x and y standard deviation is set to 0.3m
  const double HEADING_COVARIANCE = pow(0.1, 2); // heading standard deviation is set to 0.1m
};


// PointcloudObservation do not need signal_update function pointer
// its output is ICP msgs, signals go thru SourceInterface receive_odom_SE2_msg() method
class PointcloudObservation{
public:
  PointcloudObservation(std::shared_ptr<BagSource> &bag_source) {

      features_pipeline = std::make_shared<PointCloudFeaturesPipeline>();
      icp_pipeline = std::make_shared<ICPMatcherPipeline>();

      features_pipeline->publish_poles_corners = std::bind(&ICPMatcherPipeline::receive_message, &(*icp_pipeline),
                                                         std::placeholders::_1, std::placeholders::_2);

      icp_pipeline->publish_pose = std::bind(&BagSource::receive_odom_SE2_msg, &(*bag_source), std::placeholders::_1);
  }

  void receive_pointcloud(const sensor_msgs::NavSatFix::ConstPtr& pc_msg) {
        features_pipeline->receive_message(pc_msg);
  }

  std::shared_ptr<PointCloudFeaturesPipeline> features_pipeline;
  std::shared_ptr<ICPMatcherPipeline> icp_pipeline;

};



#include <sensor_msgs/NavSatFix.h>

class GNSSObservation : public ObservationModel {
public:
  GNSSObservation() : previous_east(0.), previous_north(0.) {}

  void receive_gps(double lat, double lon, int fix_status, double msg_cov, ros::Time stamp) {

      if (signal_update) {
          double east, north;
          char zone[8];

          gps_common::LLtoUTM(lat, lon, north, east, zone);

          // if this msg is the first received, or the speed is less than speed threshold, or fix_status is no_fix,
          // don't send update
          if (previous_east == 0. && previous_north == 0. ||
              pow(north-previous_north, east - previous_east)<SPEED_THRESHOLD_SQUARED ||
              fix_status == -1) {
            previous_east = east;
            previous_north = north;
            return;
          }

          double heading = atan2(north-previous_north, east - previous_east);
          previous_east = east;
          previous_north = north;

          Eigen::Matrix3d covariance;

          if (msg_cov < RTK_COVARIANCE_THRESHOLD && fix_status == 2) {
            covariance << POSITION_COVARIANCE_RTK, 0., 0.,
                0., POSITION_COVARIANCE_RTK, 0.,
                0., 0., HEADING_COVARIANCE_RTK;
          }
          else if(msg_cov <= POSITION_COVARIANCE_GNSS){
            covariance << POSITION_COVARIANCE_GNSS, 0., 0.,
                0., POSITION_COVARIANCE_GNSS, 0.,
                0., 0., HEADING_COVARIANCE_GNSS;
          }else{
              covariance << msg_cov, 0., 0.,
                  0., msg_cov, 0.,
                  0., 0., HEADING_COVARIANCE_GNSS;
          }


          Eigen::Vector3d observation(east, north, heading);

          std::string source = "gnss";
          signal_update(observation, covariance, stamp, source);
      }

  }

private:

  double previous_east, previous_north;
  const double RTK_COVARIANCE_THRESHOLD = 0.001;
  const double SPEED_THRESHOLD_SQUARED = pow(2. / 3.6); // 2km/h

  const double POSITION_COVARIANCE_GNSS = pow(2.5, 2);
  const double POSITION_COVARIANCE_RTK = pow(0.5, 2);
  const double HEADING_COVARIANCE_GNSS = pow(2. * (3.1417/180.), 2);
  const double HEADING_COVARIANCE_RTK = pow(0.5 * (3.1417/180.), 2);

};


#endif
