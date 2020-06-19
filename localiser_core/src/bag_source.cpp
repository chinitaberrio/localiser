//
// Created by stew on 25/05/20.
//

#include "bag_source.hpp"

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

      auto msg = message.instantiate<sensor_msgs::Imu>();
      if (msg != NULL && imu_topics.count(message.getTopic()) != 0)
        receive_IMU_msg(msg);

      auto msg = message.instantiate<sensor_msgs::NavSatFix>();
      if (msg && fix_topics.count(message.getTopic()) != 0) {
        receive_fix_msg(msg);
      }


      auto msg = message.instantiate<sensor_msgs::PointCloud2>();
      if (msg && pointcloud_topics.count(message.getTopic()) != 0) {
        receive_pointcloud_msg(msg);
      }


      auto msg = message.instantiate<nav_msgs::Odometry>();
      if (msg && odom_speed_topics.count(message.getTopic()) != 0){

          // in offline processing/read from bag mode, publish a tick msg for behavior tree using /zio/odometry/rear
          // max frequency is 10hz
          if(message.getTopic() == "/zio/odometry/rear"){
              if(odom_msg_count<odom_msg_reset_count){
                  odom_msg_count++;
              }else{
                  behavior_tree_pipeline = std::make_shared<BehaviorTreePipeline>();
                  behavior_tree_pipeline->receive_message(msg);
                  odom_msg_count = 0;
              }
          }

          receive_odom_speed_Msg(msg);
      }


      auto msg = message.instantiate<nav_msgs::Odometry>();
      if (msg && odom_update_topics.count(message.getTopic()) != 0){
        receive_odom_SE2_msg(msg);
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

