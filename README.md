# LIO-SAM to HD-Mapping conversion toolbox

This repository aggregates LIO-SAM and it's dependencies to for a local ROS1 Noetic deployment.

## Dockerized version of the environment

For building a dockerized version of the same environment please follow:
[ros_on_docker/LIO_SAM](https://github.com/lekston/ros_dockers/blob/dev/ros_on_docker/LIO_SAM/ROS1.md)

Applicable sections: `Getting Started`, `Docker details` and `Recording 3 (ConSLAM dataset)`

## Getting started

It is strongly recommended to use the dockerized version as it avoids modifing your local ROS installation and allows encapsulated management of all dependencies.

## Local deployment

1. Build and install `Livox-SDK/Livox-SDK2` using cmake
2. Build and install `Livox-SDK/livox_ros_driver2` using `./build.sh ROS1` included in that repo
3. Build and install `LIO-SAM` using catkin
4. Build and install `cpp_pubsub` helper node using catkin

For more details on building the above packages locally please see:
[LIO-SAM Docker file](https://github.com/lekston/ros_dockers/blob/dev/ros_on_docker/Dockerfile_ros1_lio_sam)

## Converting datasets:

(ConSLAM dataset)

*Observations*:
- LIO-SAM uses transform definitions from `params.yaml` (not from tf!)
  * LIO-SAM inputs assume that LIDAR is in ENU frame, while IMU is body NED frame (but pointing backwards)
  * LIO-SAM's IMU to LIDAR conversion requires two separate transforms `extrinsicRot` and `extrinsicRPY`
    - this is to allow support some IMUs (like Microstrain 3DM-GX5-25) which define acceleration and attitude
      in different coordinate frames.
    - for IMU used in ConSLAM dataset, both `extrinsicRot` and `extrinsicRPY` are the same (equal to `Rz_180deg`)
  * transforms for ConSLAM dataset are provided in `ConSLAM_params.yaml`
- params.yaml file is formatted during installation:
   > pre-installation example is in `ConSLAM_params.yaml`
   > post-installation requires replacing namespace `/**/ros__parameters:` with `/lio_sam:`
     * example is provided in `ConSLAM_params_local.yaml`

*ConSLAM - IMU data issues*
- no covariances in IMU messages
- `seq_01` recording is NOT continuous (jumps in time) - it is not suitable for SLAM research

*Input data*: `ConSLAM_data/seq2_recording.bag` (source: https://drive.google.com/drive/folders/1ijZs70JY9BdvobsEh-dvIqa-zsV-heL0)
Note: all \*.bags available on the source link are poorly named (all are `Copy of recording.bag`) while sequence numbers are represented in the folder names.

*Remapping of topics*:
```bash
rosbag play /data/ConSLAM_data/seq2_recording.bag --topics imu/data pp_points/synced2rgb /pp_points/synced2rgb:=/points_raw /imu/data:=/imu_raw
```

*Recording command*
```bash
rosbag record -o /data/ConSLAM_data/LIO_SAM/seq_02_results.bag /lio_sam/mapping/cloud_registered /lio_sam/mapping/path /lio_sam/mapping/odometry /odometry/imu /lio_sam/imu/path
```

*Launch LIO-SAM*
```bash
roslaunch lio_sam run.launch params_file:=/data/ConSLAM_data/LIO_SAM/ConSLAM_params_local.yaml
```

*Wait until the bag is finished*

*Convert to laz and save session.json* (HD mapping compatible)
```bash
rosrun cpp_pubsub listener /data/ConSLAM_data/LIO_SAM/seq_02_results_<latest_date>.bag /data/ConSLAM_data/LIO_SAM/session_seq_02/
```

