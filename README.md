
# the localiser

bag_input and bag_output need to be placed in different folders !!

This is currently a bug that will be fixed.

## online gps_filter rosnode
subscribe to ros topics. Localises base on GPS updates and IMU+encoder prediction.

self sufficient. does not interact with behavior tree

when bag_input arguments are left blank, gps_filter node run in online mode.


example usage:
```
In terminal 1:
roslaunch zio_maps zio_maps.launch

In terminal 2:
roslaunch localiser gps_filter.launch
```

required topics:
```
/ublox_gps/fix
/vn100/imu
/zio/odometry/rear
```


## offline gps_filter rosnode
Reads in ros msgs from a rosbag. Localises base on GPS updates and prediction.

run as fast as possible.

If want to run in near real time, set limit_playback_speed to true in gps_filter.launch

self sufficient. does not interact with behavior tree

bag_input and bag_output arguments are required to run in offline bag read mode.


example usage:
```
In terminal 1:
roslaunch zio_maps zio_maps.launch

In terminal 2:
roslaunch localiser gps_filter.launch bag_input:=/home/siqi/data/2019-02-11_Dataset_year/dataset/2019-02-11-15-42-12_Dataset_year.bag bag_output:=/home/siqi/data/2019-02-11_Dataset_year/gps_filter.bag
```

required topics in bag_input:
```
/ublox_gps/fix
/vn100/imu
/zio/odometry/rear
```


## online localiser
Reads in ros msgs from topics. Localises base on GPS and ICP matches.

behavior tree run at 2hz

if bag_output is not specified, publish to roscore topics

if bag_output is specified, write localisation result to bag file and publish to topics.



example usage:
```
In terminal 1:
roslaunch localiser localiser.launch bag_output:=/home/siqi/data/2019-02-11_Dataset_year/online_localise.bag

In terminal 2:
roslaunch localization_behavior tree.launch online:=True

# if we are playing back a bag
In terminal 3:
rosbag play sensor.bag  # /tf should be filtered out. otherwise there will be 2 conflicting vehicle paths because localiser publishes tf too
```

required topics:
```
/tf_static
/ublox_gps/fix
/velodyne/front/points
/vn100/imu
/zio/odometry/rear
```


## offline read from bag localiser
Reads in ros msgs from a rosbag. Localises base on GPS and ICP matches.

run as fast as possible.

If want to run in near real time, set limit_playback_speed to true in localiser.launch

behavior tree run at every 5 /zio/odometry/rear msgs

if bag_output is not specified, publish to topics

if bag_output is specified, write localisation result to bag file and publish to topics.

bag_input argument is required.

example usage:
```
In terminal 1:
roslaunch localiser localiser.launch bag_input:=/home/siqi/data/2019-02-11_Dataset_year/dataset/2019-02-11-15-42-12_Dataset_year.bag

In terminal 2:
roslaunch localization_behavior tree.launch online:=False

```

required topics in bag_input:
```
/ublox_gps/fix
/velodyne/front/points
/vn100/imu
/zio/odometry/rear
```
