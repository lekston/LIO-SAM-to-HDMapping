#!/usr/bin/env python3

import itertools
import rospy
import numpy as np
import sys
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ImuTimeStatsMonitor:
    def __init__(self):
        # Initialize ROS node first
        rospy.init_node('imu_analyzer', anonymous=True)

        # Process command line ROS parameters after node initialization
        self._process_ros_params()

        # Get parameters with defaults
        self.input_topic = rospy.get_param('~input_topic', '/imu_raw')
        self.msg_type = rospy.get_param('~msg_type', 'imu')  # 'imu', 'odom', or 'auto'

        # Setup subscriber based on message type
        self._setup_subscriber()

        self._buffer_len = 20
        self._msgs = []
        self._start_time = 0

        rospy.loginfo("IMU monitor started:")
        rospy.loginfo("  Input topic: %s", self.input_topic)
        rospy.loginfo("  Message type: %s", self.msg_type)
        rospy.loginfo("  Size of averaging window: %s", self._buffer_len)

    def _setup_subscriber(self):
        """Setup subscriber based on message type parameter"""
        if self.msg_type.lower() == 'imu':
            self.imu_sub = rospy.Subscriber(self.input_topic, Imu, self._callback_wrapper, queue_size=10)
            rospy.loginfo("Subscribed to %s as sensor_msgs/Imu", self.input_topic)
        elif self.msg_type.lower() == 'odom':
            self.imu_sub = rospy.Subscriber(self.input_topic, Odometry, self._callback_wrapper, queue_size=10)
            rospy.loginfo("Subscribed to %s as nav_msgs/Odometry", self.input_topic)
        elif self.msg_type.lower() == 'auto':
            # Try to auto-detect by subscribing to both and see which one receives messages first
            rospy.loginfo("Auto-detecting message type for topic %s...", self.input_topic)
            self._auto_detect_msg_type()
        else:
            rospy.logerr("Invalid msg_type parameter: %s. Use 'imu', 'odom', or 'auto'", self.msg_type)
            sys.exit(1)

    def _auto_detect_msg_type(self):
        """Auto-detect message type by trying both subscribers"""
        self._detection_complete = False

        # Create temporary subscribers for detection
        self._temp_imu_sub = rospy.Subscriber(self.input_topic, Imu, self._detect_imu, queue_size=1)
        self._temp_odom_sub = rospy.Subscriber(self.input_topic, Odometry, self._detect_odom, queue_size=1)

        # Wait for detection or timeout
        timeout = rospy.Time.now() + rospy.Duration(5.0)  # 5 second timeout
        rate = rospy.Rate(10)

        while not self._detection_complete and rospy.Time.now() < timeout and not rospy.is_shutdown():
            rate.sleep()

        # Clean up temporary subscribers
        self._temp_imu_sub.unregister()
        self._temp_odom_sub.unregister()

        if not self._detection_complete:
            rospy.logerror("Could not auto-detect message type.")
            sys.exit(1)

    def _detect_imu(self, msg):
        """Callback for IMU message detection"""
        if not self._detection_complete:
            self._detection_complete = True
            self.msg_type = 'imu'
            rospy.loginfo("Detected sensor_msgs/Imu messages on %s", self.input_topic)
            self.imu_sub = rospy.Subscriber(self.input_topic, Imu, self._callback_wrapper, queue_size=10)

    def _detect_odom(self, msg):
        """Callback for Odometry message detection"""
        if not self._detection_complete:
            self._detection_complete = True
            self.msg_type = 'odom'
            rospy.loginfo("Detected nav_msgs/Odometry messages on %s", self.input_topic)
            self.imu_sub = rospy.Subscriber(self.input_topic, Odometry, self._callback_wrapper, queue_size=10)

    def _callback_wrapper(self, msg):
        """Wrapper to handle both message types"""
        if isinstance(msg, Imu):
            self.imu_callback(msg)
        elif isinstance(msg, Odometry):
            self.odom_callback(msg)

    def _extract_data_from_imu(self, msg):
        """Extract timestamp and quaternion from IMU message"""
        return msg.header.stamp, [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    def _extract_data_from_odom(self, msg):
        """Extract timestamp and quaternion from Odometry message"""
        orientation = msg.pose.pose.orientation
        return msg.header.stamp, [orientation.x, orientation.y, orientation.z, orientation.w]

    def odom_callback(self, msg):
        """Handle Odometry messages by converting them to IMU-like format"""
        # Create a simplified message structure for consistent processing
        class SimpleMsg:
            def __init__(self, header_stamp, orientation):
                self.header = type('obj', (object,), {'stamp': header_stamp})()
                self.orientation = orientation

        # Extract orientation from odometry
        orientation = msg.pose.pose.orientation

        # Create simplified message for processing
        simple_msg = SimpleMsg(msg.header.stamp, orientation)

        # Process using the same logic as IMU
        self.imu_callback(simple_msg)

    def _process_ros_params(self):
        """Process ROS-style command line arguments and set them as ROS parameters"""
        for arg in sys.argv[1:]:
            if arg.startswith('_') and ':=' in arg:
                param_name, param_value = arg.split(':=', 1)
                # Remove leading underscore and make it a private parameter
                param_name = f"~{param_name[1:]}"

                # Try to convert to appropriate type
                if param_value.lower() in ['true', 'false']:
                    param_value = param_value.lower() == 'true'
                elif param_value.replace('.', '').replace('-', '').isdigit():
                    param_value = float(param_value) if '.' in param_value else int(param_value)

                rospy.set_param(param_name, param_value)
                rospy.loginfo(f"Set parameter {param_name} = {param_value}")

    def timestamp(self, header_time_stamp):
        ts, tn = header_time_stamp.secs, header_time_stamp.nsecs
        return ts + tn*1e-9

    def _pairwise(self, iterable):
        a, b = itertools.tee(iterable)
        next(b, None)
        return zip(a, b)

    def _time_diff(self, m1, m2):
        t1 = self.timestamp(m1.header.stamp)
        t2 = self.timestamp(m2.header.stamp)
        return t2-t1

    def imu_callback(self, msg):
        if self._start_time == 0:
            self._start_time = msg.header.stamp.secs
        msg.header.stamp.secs -= self._start_time
        self._msgs.append(msg)
        if len(self._msgs) > self._buffer_len:
            self._msgs.pop(0)
            # import pdb; pdb.set_trace()
            time_diffs = list(map(lambda p: self._time_diff(p[0], p[1]), self._pairwise(self._msgs)))
            avg = np.average(time_diffs)
            median = np.median(time_diffs)
            std = np.std(time_diffs)
            td_max = max(time_diffs)
            td_min = min(time_diffs)

            q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            q_norm = np.linalg.norm(q)

            rospy.loginfo(
                    f"Time diffs: last {time_diffs[-1]:.6f} [sec] (q_norm: {q_norm:.6f})\n"
                    f"(windowed: min {td_min:.6f}, max {td_max:.6f}, avg {avg:.6f}, median {median:.6f}, std {std:.6f})"
            )


if __name__ == '__main__':
    try:
        remapper = ImuTimeStatsMonitor()
        rospy.loginfo("IMU time stats monitor node is running...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
