#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include "linear_filter.hpp"



class MotionModel
{
public:
    MotionModel();

    void calculate_pose_increment(ros::Time stamp);
    void receive_yaw_rate(double yaw_rate, double variance, ros::Time stamp);
    void receive_pitch(double pitch, double variance, ros::Time stamp);
    void receive_speed(double speed, double variance, ros::Time stamp);

private:
    Eigen::Vector3d vehicle_model(double distance_travelled, double delta_heading);

    // use hard coded uncertainty values from python code
    const float VELOCITY_NOISE = 2.5; // m/s
    float YAWRATE_NOISE = 1.5 * (3.1415 / 180.0); // deg/s

   //! The absolute measurement of the robot pitch
    double measured_pitch;
    double measured_pitch_variance; // not actually used

    //! The measurement of the robots speed
    double measured_speed;
    double measured_speed_variance; // not actually used

    //! The measurement of the robots yaw rate
    double measured_yaw_rate;
    double measured_yaw_rate_variance; // not actually used
};

#endif // MOTIONMODEL_H



