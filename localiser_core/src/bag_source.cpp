//
// Created by stew on 25/05/20.
//

#include "bag_source.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>



BagSource::BagSource(){
    // max frequency is 10hz
    float FREQENCY = 2.;// set behavior tree tick to 2hz
    odom_msg_reset_count = 10./FREQENCY;


}



void BagSource::MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {

//    std::string const& topic = message.getTopic();
//    ros::Time const& header_time = message.getTime();


//  auto tf_msg = message.instantiate<tf2_msgs::TFMessage>();
//  if (tf_msg) {
//    std::cout << "TF: " << message.getTopic() << std::endl;
//    publisher.publish(message);
//  }

      auto imu_msg = message.instantiate<sensor_msgs::Imu>();
      if (imu_msg && imu_topics.count(message.getTopic()) != 0){
        receive_IMU_msg(imu_msg);
        return;
      }

      auto fix_msg = message.instantiate<sensor_msgs::NavSatFix>();
      if (fix_msg && fix_topics.count(message.getTopic()) != 0) {
        receive_fix_msg(fix_msg);
        return;
      }


      auto pc_msg = message.instantiate<sensor_msgs::PointCloud2>();
      if (pc_msg && pointcloud_topics.count(message.getTopic()) != 0) {
        receive_pointcloud_msg(pc_msg);
        return;
      }


      auto odom_speed_msg = message.instantiate<nav_msgs::Odometry>();
      if (odom_speed_msg && odom_speed_topics.count(message.getTopic()) != 0){

          // in offline processing/read from bag mode, publish a tick msg for behavior tree using /zio/odometry/rear
          // max frequency is 10hz
          if(message.getTopic() == "/zio/odometry/rear"){
              if(odom_msg_count<odom_msg_reset_count){
                  odom_msg_count++;
              }else{
                  behavior_tree_pipeline = std::make_shared<BehaviorTreePipeline>();
                  behavior_tree_pipeline->receive_message(odom_speed_msg);
                  odom_msg_count = 0;
              }
          }

          receive_odom_speed_msg(odom_speed_msg);
          return;
      }


      auto odom_SE2_msg = message.instantiate<nav_msgs::Odometry>();
      if (odom_SE2_msg && odom_SE2_topics.count(message.getTopic()) != 0){
        receive_odom_SE2_msg(odom_SE2_msg);
        return;
      }

}



void BagSource::ReadBag(std::string bag_file) {


  //private_nh.setParam("bag_file", bag_file_name);

  bag_file_name = bag_file;

  std::cout << bag_file_name << std::endl;

  this->init_playback();

  this->ReadFromBag();

  std::cout << this->bag_file_name << std::endl;

}

