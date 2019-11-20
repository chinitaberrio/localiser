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


class Observer {
public:
  Observer();

  //! Perform the prediction
  std::function<void(Eigen::Vector3d&, Eigen::Matrix3d&, ros::Time)> perform_update;



  /// define parameters such as min/max reliable measurements, lost parameters, etc
};


#include <nav_msgs/Odometry.h>

class ICPObservation : public Observer {
public:
  ICPObservation() : Observer() {}

  void receive_message(const nav_msgs::Odometry::ConstPtr& msg) {
    if (perform_update) {

      tf2::Quaternion orientation_tf;
      tf2::convert(msg->pose.pose.orientation, orientation_tf);

      tf2::Matrix3x3 mat(orientation_tf);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);
      // covariance [4] is y-orientation

      // covariance is the diagonal of term 0 (x) term 1 (y) and term 5 (heading)
      Eigen::Matrix3d observation_covariance;
      observation_covariance << msg->pose.covariance[0 + 0 * 6], 0., 0.,
          0., msg->pose.covariance[1 + 1 * 6], 0.,
          0., 0., msg->pose.covariance[5 + 5 * 6];

      Eigen::Vector3d observation_vector;
      observation_vector << msg->pose.pose.position.x,
                            msg->pose.pose.position.y,
                            yaw;

      // covariance [0] is x-twist-linear
      perform_update(observation_vector,
                     observation_covariance,
                     msg->header.stamp);
    }
  }
};




#include <sensor_msgs/NavSatFix.h>

class GNSSObservation : public Observer {
public:
  GNSSObservation() : Observer(), previous_east(0.), previous_north(0.) {}

  const double RTK_COVARIANCE_THRESHOLD = 0.001;

  const double POSITION_COVARIANCE_GNSS = pow(2.5, 2);
  const double POSITION_COVARIANCE_RTK = pow(0.5, 2);
  const double HEADING_COVARIANCE = pow(2. * (3.1417/180.), 2);

  void receive_message(const sensor_msgs::NavSatFix::ConstPtr& msg) {
      double east, north;
      char zone[8];

      gps_common::LLtoUTM(msg->latitude, msg->longitude, north, east, zone);

      double heading = 0.;

      if (previous_east == 0. && previous_north == 0.) {
        heading = atan2(north-previous_north, east - previous_east);
        previous_east = east;
        previous_north = north;
        return;
      }

      heading = atan2(north-previous_north, east - previous_east);
      previous_east = east;
      previous_north = north;

      Eigen::Matrix3d covariance;

      if (msg->position_covariance[0] < RTK_COVARIANCE_THRESHOLD) {
        covariance << POSITION_COVARIANCE_RTK, 0., 0.,
            0., POSITION_COVARIANCE_RTK, 0.,
            0., 0., POSITION_COVARIANCE_RTK;
      }
      else {
        covariance << POSITION_COVARIANCE_GNSS, 0., 0.,
            0., POSITION_COVARIANCE_GNSS, 0.,
            0., 0., HEADING_COVARIANCE;
      }

      if (perform_update) {
        Eigen::Vector3d observation(east, north, heading);
        perform_update(observation, covariance, msg->header.stamp);
      }

  }

private:
  double previous_east, previous_north;
};


#endif
