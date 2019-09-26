#ifndef bitmap_output_h
#define bitmap_output_h

// include messages to write to bag file
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include "point_xyzir.h"
#include "point_xyzirc.h"


/*!
 * \brief Class to publish the ros messages
 *
 */

class BitmapOutput {
public:
  BitmapOutput();
  ~BitmapOutput();

  void Initialise(std::string bag_file);

  void publish_odom(nav_msgs::Odometry &msg, std::string topic_name);
  void publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name);

  void receive_message(const pcl::PointCloud<pcl::PointXYZIRC>::Ptr &pointcloud);

};




#endif
