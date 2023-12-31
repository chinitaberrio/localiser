# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(localiser)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug)
ENDIF()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")

#set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(MRPT REQUIRED  gui obs slam ros1bridge)
 
if (MRPT_VERSION VERSION_LESS 2.0.0)
  set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  gps_common
  custom_point_types
  dataset_tools
  h264_bag_playback
  cv_bridge
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  roslaunch
  rosbag
  tf2
  tf2_geometry_msgs
)

#find mrpt packages
find_package(MRPT REQUIRED  gui obs slam ros1bridge)

#if (MRPT_VERSION VERSION_LESS 2.0.0)
#        set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")
#endif()

find_package(OpenCV 3 REQUIRED)

find_package(Eigen3 REQUIRED)
#find_package(GTSAM REQUIRED)



#find_package(G2O REQUIRED)
# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
SET(G2O_ROOT /usr/local/include/g2o )
find_package(G2O)
find_package(CSparse)

find_package(Boost COMPONENTS system filesystem REQUIRED)


find_package(PCL REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Generate services in the 'srv' folder
add_service_files(
        FILES
        instruct_localiser.srv
        reset_localiser.srv

)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS nav_msgs sensor_msgs std_msgs sensor_msgs rosbag

  INCLUDE_DIRS include
)

###########
## Build ##
###########

# eigen3 is header only
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CSPARSE_INCLUDE_DIR}
  ${G2O_INCLUDE_DIRS}
  ${MRPT_LIBS}
#  gtsam
  include/${PROJECT_NAME}
  include/localiser_core
  include
  ${PCL_INCLUDE_DIRS}
)

# collect all g2o libs
set(g2o_LIB_collection
    g2o_core
    #g2o_types_slam3d
    g2o_stuff
    #g2o_types_slam3d_addons
    #g2o_hierarchical
    #g2o_simulator
    #g2o_types_data
    g2o_types_slam2d
    #g2o_types_sba
    #g2o_types_sim3
    #g2o_types_icp
    #g2o_types_sclam2d g2o_types_slam2d_addons
    #g2o_solver_pcg g2o_solver_dense g2o_solver_structure_only
    g2o_solver_csparse g2o_csparse_extension #g2o_solver_slam2d_linear
    #g2o_solver_cholmod g2o_solver_eigen
)

add_library(localiser_core
#    src/ros_localiser.cpp
    src/localiser_core/bag_source.cpp
    src/localiser_core/bag_destination.cpp
    src/localiser_core/destination_interface.cpp
    src/localiser_core/source_interface.cpp
    src/localiser_core/observation_model.cpp
    src/localiser_core/motion_model.cpp
    src/localiser_core/linear_filter.cpp
#    src/graph_optimiser.cpp
#    src/gtsam_optimiser.cpp
    src/localiser_core/publisher.cpp
)


TARGET_LINK_LIBRARIES(localiser_core
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
#    icp_map
#    simple_graph
 ${MRPT_LIBS}
# ${PCL_LIBRARIES}
#    ${CSPARSE_LIBRARY} # csparse is used for graph optimization
#    ${g2o_LIB_collection}
# ${g2o_LIB_collection}
 #gtsam
# ${GTSAM_LIBS}
  ${Boost_SYSTEM_LIBRARY}

)
add_executable(${PROJECT_NAME}
    src/ros_localiser.cpp
  #  src/icp_matcher_pipeline.cpp
    #src/icp_map_maker.cpp
)


TARGET_LINK_LIBRARIES(${PROJECT_NAME}
    localiser_core
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(gps_filter
    src/gps_filter.cpp
  src/localiser_core/bag_source.cpp
  src/localiser_core/bag_destination.cpp
  src/localiser_core/destination_interface.cpp
  src/localiser_core/source_interface.cpp
  src/localiser_core/observation_model.cpp
  src/localiser_core/motion_model.cpp
  src/localiser_core/linear_filter.cpp
  #    src/graph_optimiser.cpp
  #    src/gtsam_optimiser.cpp
  src/localiser_core/publisher.cpp

)


TARGET_LINK_LIBRARIES(gps_filter
    localiser_core
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}

)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS
  ${PROJECT_NAME} localiser_core gps_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  #launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )

#############
## Testing ##
#############
# Add testing for *.launch files
#roslaunch_add_file_check(launch)
