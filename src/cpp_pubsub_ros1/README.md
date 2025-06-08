# Intro

This coversion package is provided to allow conversion of LIO-SAM bags to HDMapping compatible format.

The LIO-SAM output topics are hardcoded to:
- `/lio_sam/mapping/cloud_registered` (for sensor_msgs::PointCloud2 messages)
- `/odometry/imu` (for nav_msgs::Odometry messages)

If needed please remap the topics when running this tool.

# Building cpp_pubsub

This ROS1 version based on same tool prepared for FAST-LIVO based on: https://github.com/marcinmatecki/fast-livo-to-hdmapping

It was updated to use livox_ros_driver2

## dependencies
```bash
sudo apt install nlohmann-json3-dev
```

```bash
git submodule update --init --recursive
```

Important: source the livox ROS1 drivers before building the cpp_pubsub package
```bash
source <path_to_ws_with_livox_ros_driver2>/catkin_ws/devel/setup.bash
```

## run
```bash
source /livox_sdk/catkin_ws/devel/setup.bash  # livox_ros_driver2
source /catkin_ws/devel/setup.bash  # LIO-SAM

# need to provide both the bag file and the output directory
rosrun cpp_pubsub listener /data/LIO_SAM/walking_dataset_cloud_registered_2025-05-02-05-35-35.bag /data/LIO_SAM/session_HDmapping
```

## debug

### building with debug symbols
```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### running with gdb
```bash
rosrun --prefix 'gdb --args' cpp_pubsub listener /data/LIO_SAM/walking_dataset_cloud_registered_2025-05-02-05-35-35.bag
/data/LIO_SAM/session_HDmapping
```

# Related sources

- HDMapping:
  * `./apps/split_multi_livox/split_multi_livox.cpp`
  * `./apps/lidar_odometry_step_1/lidar_odometry.h`
  * `./apps/lidar_odometry_step_1/lidar_odometry.cpp`
  * `./core/include/point_clouds.h`
  * `./core/src/point_clouds.cpp`

# TODOs:
- consider a more meaningful name for cpp_pubsub (e.g.: bag2laz)
- add parameter handling for cpp_pubsub (for easier topic remapping)
- streamline ROS1 and ROS2 versions of the cpp_pubsub:
  - ROS1 source: https://github.com/marcinmatecki/fast-livo-to-hdmapping
  - ROS2 source: https://github.com/marcinmatecki/kiss-icp-to-hdmapping