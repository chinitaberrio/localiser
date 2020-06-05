#ifndef localiser_input_h
#define localiser_input_h

#include <ros/ros.h>

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "observer.hpp"
#include "predictor.hpp"


/*!
 * \brief The class that manages the localisation process
 *
 */

class LocaliserInput {
public:
  LocaliserInput();

  //! Pass the update vector to the localisation method
  std::function<void(Eigen::Vector3d&, Eigen::Matrix3d&, ros::Time, std::string &source)> perform_update;

  //! Pass the prediction vector to the localisation method
  std::function<void(Eigen::Vector2d&, Eigen::Matrix2d&, ros::Time)> perform_prediction;


  void SetPitch(double pitch, double variance, ros::Time stamp);
  void SetSpeed(double speed, double variance, ros::Time stamp);
  void SetYawRate(double yaw_rate, double variance, ros::Time stamp);

  //! Perform a prediction update step
  void Predict(ros::Time stamp);

  //! Perform the update
  void Update(Eigen::Vector3d &observation, Eigen::Matrix3d &covariance, ros::Time stamp, std::string &source);

  std::vector<std::shared_ptr<Predictor>> predictors;
  std::vector<std::shared_ptr<Observer>> observers;

private:

  //! The absolute measurement of the robot pitch
  double measured_pitch;

  //! The measurement of the robots speed
  double measured_speed;
  double measured_speed_variance;

  //! The measurement of the robots yaw rate
  double measured_yaw_rate;
  double measured_yaw_rate_variance;

};



#endif
