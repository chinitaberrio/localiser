#ifndef publisher_h
#define publisher_h

#include <map>

#include <ros/ros.h>

// include messages to publish

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

//
//#include <Eigen/Core>
//#include <Eigen/StdVector>
//
//#include <vector>



/*!
 * \brief Class to publish the ros messages
 *
 */

class Publisher {
public:
  Publisher()  {}

  void Initialise();

  void publish_odom(nav_msgs::Odometry &msg, std::string topic_name);
  void publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name);

  std::map<std::string, ros::Publisher> publishers;
  ros::Publisher pub;
};




#endif
