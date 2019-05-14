#include "localisation_method.hpp"

#include <ostream>
#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/StdVector>




LocalisationMethod::LocalisationMethod() :
    map_state(Eigen::Vector3d(0.,0.,0.)),
    odom_state(Eigen::Vector3d(0.,0.,0.)),
    previous_prediction_stamp(ros::Time(0.)),
    previous_observation_stamp(ros::Time(0.))
{}
