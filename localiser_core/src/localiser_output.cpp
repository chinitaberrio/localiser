#include "localiser_output.hpp"

#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>


LocaliserOutput::LocaliserOutput() :
  datum_x(0.0),
  datum_y(0.0)
{}



void
LocaliserOutput::PublishStatistics(Eigen::Vector3d &innovation, Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time stamp) {

  dataset_tools::LocaliserStats stats_msg;
  stats_msg.header.stamp = stamp;

  stats_msg.innovation.x = innovation(0);
  stats_msg.innovation.y = innovation(1);
  stats_msg.innovation.yaw = innovation(2);

  stats_msg.confidence.x = confidence(0);
  stats_msg.confidence.y = confidence(1);
  stats_msg.confidence.yaw = confidence(2);

  stats_msg.covariance[0] = covariance(0);
  stats_msg.covariance[1] = covariance(1);
  stats_msg.covariance[2] = covariance(2);
  stats_msg.covariance[3] = covariance(3);
  stats_msg.covariance[4] = covariance(4);
  stats_msg.covariance[5] = covariance(5);
  stats_msg.covariance[6] = covariance(6);
  stats_msg.covariance[7] = covariance(7);
  stats_msg.covariance[8] = covariance(8);


  if (publish_stats) {
    publish_stats(stats_msg, "stats");
  }
}



void
LocaliserOutput::PublishOdometry(Eigen::Vector3d &odometry, Eigen::Matrix3d &covariance, ros::Time stamp) {

  // generate an global odometry pose message
  nav_msgs::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "odom";
  msg.pose.pose.position.x = odometry[0];
  msg.pose.pose.position.y = odometry[1];

  tf2::Quaternion orientation;
  orientation.setRPY(0., 0., odometry[2]);  // Create this quaternion from roll/pitch/yaw (in radians)
  msg.pose.pose.orientation.x = orientation[0];
  msg.pose.pose.orientation.y = orientation[1];
  msg.pose.pose.orientation.z = orientation[2];
  msg.pose.pose.orientation.w = orientation[3];

  if (publish_odom) {
    publish_odom(msg, "odometry");
  }

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(odometry[0], odometry[1], 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, odometry[2]);
  transform.setRotation(q);
  tf::StampedTransform odom_baselink_tf(transform, stamp, "odom", "base_link");
/*
  transform_broadcaster.sendTransform(odom_baselink_tf);

  tf2_msgs::TFMessage tf_pub_message;
  geometry_msgs::TransformStamped geom_tf;
  geom_tf.transform.rotation.x = q[0];
  geom_tf.transform.rotation.y = q[1];
  geom_tf.transform.rotation.z = q[2];
  geom_tf.transform.rotation.w = q[3];
  geom_tf.transform.translation.x = odometry[0];
  geom_tf.transform.translation.y = odometry[1];
  geom_tf.transform.translation.z = 0.0;
  geom_tf.header.stamp = stamp;
  geom_tf.header.frame_id = "odom";
  geom_tf.child_frame_id = "base_link";
  tf_pub_message.transforms.push_back(geom_tf);
*/
  if (publish_tf) {
    publish_tf(odom_baselink_tf, "/tf");
  }

  //transform_buffer.setTransform(odom_baselink_tf, "zio");

}



void
LocaliserOutput::PublishMap(Eigen::Vector3d &map_estimate, Eigen::Matrix3d &covariance, Eigen::Vector3d &odom_delta, ros::Time stamp) {

  // generate an global odometry pose message
  nav_msgs::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.pose.pose.position.x = map_estimate[0];
  msg.pose.pose.position.y = map_estimate[1];

  tf2::Quaternion orientation;
  orientation.setRPY(0., 0., map_estimate[2]);  // Create this quaternion from roll/pitch/yaw (in radians)
  msg.pose.pose.orientation.x = orientation[0];
  msg.pose.pose.orientation.y = orientation[1];
  msg.pose.pose.orientation.z = orientation[2];
  msg.pose.pose.orientation.w = orientation[3];

  if (publish_odom) {
    publish_odom(msg, "map");
  }

  // generate an global navsatfix message
  double lat, lon;
  gps_common::UTMtoLL(map_estimate[1], map_estimate[0], "56H", lat, lon);

  sensor_msgs::NavSatFix fix_msg;
  fix_msg.header.frame_id = "map";
  fix_msg.header.stamp = stamp;
  fix_msg.latitude = lat;
  fix_msg.longitude = lon;

  if (publish_fix) {
    publish_fix(fix_msg, "fix");
  }

  if (datum_x == 0. || datum_y == 0.) {

    tf::StampedTransform transform;
    try {
      transform_listener.lookupTransform("utm", "map", ros::Time(0), transform);
      datum_x = transform.getOrigin().x();
      datum_y = transform.getOrigin().y();

      // todo: output this transform: include in the new bag
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }
  }

  if (datum_x != 0. && datum_y != 0.) {
    // publish the map transform
    tf::Transform transform;
    tf::Vector3 map_position(map_estimate[0], map_estimate[1], 0.0);
    tf::Vector3 map_odom_delta(odom_delta[0], odom_delta[1], 0.0);
    tf::Vector3 datum(datum_x, datum_y, 0.0);





    // map to odom transform is translation only, no rotation
    transform.setOrigin(map_position - map_odom_delta - datum);
    tf::Quaternion q;
    q.setRPY(0., 0., 0.);
    transform.setRotation(q);
    tf::StampedTransform map_odom_tf(transform, stamp, "map", "odom");
/*
    transform_broadcaster.sendTransform(map_odom_tf);


    tf2_msgs::TFMessage tf_pub_message;
    geometry_msgs::TransformStamped geom_tf;
    geom_tf.transform.rotation.x = q[0];
    geom_tf.transform.rotation.y = q[1];
    geom_tf.transform.rotation.z = q[2];
    geom_tf.transform.rotation.w = q[3];
    geom_tf.transform.translation.x = transform.getOrigin()[0];
    geom_tf.transform.translation.y = transform.getOrigin()[1];
    geom_tf.transform.translation.z = transform.getOrigin()[2];
    geom_tf.header.stamp = stamp;
    geom_tf.header.frame_id = "map";
    geom_tf.child_frame_id = "odom";
    tf_pub_message.transforms.push_back(geom_tf);
*/
    if (publish_tf) {
      publish_tf(map_odom_tf, "/tf");
    }

    //transform_buffer.setTransform(map_odom_tf, "zio");
  }
}
