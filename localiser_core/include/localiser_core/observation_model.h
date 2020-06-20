#ifndef observer_h
#define observer_h

#include <cmath>
#include <string>

#include <gps_common/conversions.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
//#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <dataset_tools/point_cloud_features_pipeline.hpp>
#include <dataset_tools/icp_matcher_pipeline.hpp>

#include "bag_source.h"

/*!
 * \brief The base class for providing observations
 * Class type that gives an observation of the current robot pose in the
 * global frame of reference
 */


class ObservationModel {
public:
    ObservationModel(){}

  std::function<void(Eigen::Vector3d&, Eigen::Matrix3d&, ros::Time, std::string&)> signal_update;



  /// define parameters such as min/max reliable measurements, lost parameters, etc
};


#include <nav_msgs/Odometry.h>

class ICPObservation : public ObservationModel {
public:
  ICPObservation() {}

  void receive_ICP(double x, double y, double heading, ros::Time stamp);

private:

  const double POSITION_COVARIANCE = pow(0.3, 2); // x and y standard deviation is set to 0.3m
  const double HEADING_COVARIANCE = pow(0.1, 2); // heading standard deviation is set to 0.1m
};


// PointcloudObservation do not need signal_update function pointer
// its output is ICP msgs, signals go thru SourceInterface receive_odom_SE2_msg() method
class PointcloudObservation{
public:
  PointcloudObservation(std::shared_ptr<BagSource> &bag_source) ;

  void receive_pointcloud(const sensor_msgs::PointCloud2::ConstPtr& pc_msg) ;

  std::shared_ptr<PointCloudFeaturesPipeline> features_pipeline;
  std::shared_ptr<ICPMatcherPipeline> icp_pipeline;

};



#include <sensor_msgs/NavSatFix.h>

class GNSSObservation : public ObservationModel {
public:
  GNSSObservation() : previous_east(0.), previous_north(0.) {}

  void receive_gps(double lat, double lon, int fix_status, double msg_cov, ros::Time stamp);

private:

  double previous_east, previous_north;
  const double RTK_COVARIANCE_THRESHOLD = 0.001;
  const double SPEED_THRESHOLD_SQUARED = pow(2. / 3.6, 2); // 2km/h

  const double POSITION_COVARIANCE_GNSS = pow(2.5, 2);
  const double POSITION_COVARIANCE_RTK = pow(0.5, 2);
  const double HEADING_COVARIANCE_GNSS = pow(2. * (3.1417/180.), 2);
  const double HEADING_COVARIANCE_RTK = pow(0.5 * (3.1417/180.), 2);

};


#endif
