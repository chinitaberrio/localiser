#include "destination_interface.h"

DestinationInterface::DestinationInterface()
{

}



dataset_tools::LocaliserStats
BagOutput::receive_stats2msg(Eigen::Vector3d &observation, Eigen::Vector3d &innovation,
            Eigen::Matrix3d &covariance, Eigen::Vector3d &confidence, ros::Time stamp, std::string &source){


      dataset_tools::LocaliserStats stats_msg;
      stats_msg.header.stamp = stamp;

      stats_msg.source = source;

      stats_msg.observation.x = observation(0);
      stats_msg.observation.y = observation(1);
      stats_msg.observation.yaw = observation(2);

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

    return stats_msg;
}


nav_msgs::Odometry
BagOutput::receive_odom2msg(std::string &frame_id, std::string &topic_name, Eigen::Vector3d &SE2_estimate,
                                   Eigen::Matrix3d &covariance, ros::Time &stamp) {


        nav_msgs::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = frame_id;
        msg.pose.pose.position.x = SE2_estimate[0];
        msg.pose.pose.position.y = SE2_estimate[1];

        tf2::Quaternion orientation;
        orientation.setRPY(0., 0., SE2_estimate[2]);  // Create this quaternion from roll/pitch/yaw (in radians)
        msg.pose.pose.orientation.x = orientation[0];
        msg.pose.pose.orientation.y = orientation[1];
        msg.pose.pose.orientation.z = orientation[2];
        msg.pose.pose.orientation.w = orientation[3];

      return msg;
}


tf2_msgs::TFMessage
BagOutput::receive_map_tf2msg(Eigen::Vector3d &map_SE2_estimate, ros::Time &stamp) {


    if(stamp != odom_stamp){
        ROS_ERROR_STREAM("publishing map tf without correspondant odom tf");
    }

    if (datum_x == 0. || datum_y == 0.) {

      tf::StampedTransform transform;
      try {
        transform_listener.lookupTransform("utm", "map", ros::Time(0), transform);
        datum_x = transform.getOrigin().x();
        datum_y = transform.getOrigin().y();
      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
      }

    }


    if (datum_x != 0. && datum_y != 0.) {
      // publish the map transform
      tf::Transform transform;
      tf::Vector3 base_in_utm(map_SE2_estimate[0], map_SE2_estimate[1], 0.0);
      tf::Vector3 base_in_odom_position(odom_state[0], odom_state[1], 0.0);
      tf::Vector3 datum(datum_x, datum_y, 0.0);

      // map to odom transform is translation only, no rotation
      transform.setOrigin(base_in_utm - datum - base_in_odom_position);
      tf::Quaternion q;
      q.setRPY(0., 0., 0.);
      transform.setRotation(q);
      tf::StampedTransform map_odom_tf(transform, stamp, "map", "odom");

    }

    // TODO: this could prob be simplified

    tf2_msgs::TFMessage tf_pub_message;
    geometry_msgs::TransformStamped geom_tf;
    geom_tf.transform.rotation.x = map_odom_tf.getRotation()[0];
    geom_tf.transform.rotation.y = map_odom_tf.getRotation()[1];
    geom_tf.transform.rotation.z = map_odom_tf.getRotation()[2];
    geom_tf.transform.rotation.w = map_odom_tf.getRotation()[3];
    geom_tf.transform.translation.x = map_odom_tf.getOrigin()[0];
    geom_tf.transform.translation.y = map_odom_tf.getOrigin()[1];
    geom_tf.transform.translation.z = map_odom_tf.getOrigin()[2];
    geom_tf.header.stamp = map_odom_tf.stamp_;
    geom_tf.header.frame_id = map_odom_tf.frame_id_;
    geom_tf.child_frame_id = map_odom_tf.child_frame_id_;
    tf_pub_message.transforms.push_back(geom_tf);

    return tf_pub_message;
}

tf2_msgs::TFMessage
BagOutput::receive_odom_tf2msg(Eigen::Vector3d &odom_SE2_estimate, ros::Time &stamp) {
    odom_state = SE2_estimate;
    odom_time = stamp;

    stamp -= ros::Duration(0.0000001);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom_SE2_estimate[0], odom_SE2_estimate[1], 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, odom_SE2_estimate[2]);
    transform.setRotation(q);
    tf::StampedTransform odom_baselink_tf(transform, stamp, "odom", "base_link");

    tf2_msgs::TFMessage tf_pub_message;
    geometry_msgs::TransformStamped geom_tf;
    geom_tf.transform.rotation.x = msg.getRotation()[0];
    geom_tf.transform.rotation.y = msg.getRotation()[1];
    geom_tf.transform.rotation.z = msg.getRotation()[2];
    geom_tf.transform.rotation.w = msg.getRotation()[3];
    geom_tf.transform.translation.x = msg.getOrigin()[0];
    geom_tf.transform.translation.y = msg.getOrigin()[1];
    geom_tf.transform.translation.z = msg.getOrigin()[2];
    geom_tf.header.stamp = stamp;
    geom_tf.header.frame_id = msg.frame_id_;
    geom_tf.child_frame_id = msg.child_frame_id_;
    tf_pub_message.transforms.push_back(geom_tf);

    return tf_pub_message;
}

