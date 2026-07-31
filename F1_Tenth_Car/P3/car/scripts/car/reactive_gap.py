#!/usr/bin/env python
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# Constants
WINDOW_SIZE = 5
RANGE_THRESHOLD = 1.0
BUBBLE_RADIUS = 10
ANGLE_LIMIT = 70
VELOCITY_LIMITS = [(10, 1.0), (20, 0.7)]


class ReactiveFollowGap:
    def __init__(self):
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        # self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size=10)
        self.drive_pub = rospy.Publisher(
            "/vesc/high_level/ackermann_cmd_mux/input/nav_0",
            AckermannDriveStamped,
            queue_size=10,
        )

    def preprocess_lidar(self, data):
        orig_proc_ranges = np.array(data.ranges)
        orig_proc_ranges[
            np.isnan(orig_proc_ranges)
            | np.isinf(orig_proc_ranges)
            | (orig_proc_ranges < data.range_min)
        ] = 0
        orig_proc_ranges[orig_proc_ranges < data.range_min] = data.range_min
        orig_proc_ranges[
            (orig_proc_ranges > data.range_max) | (orig_proc_ranges > 3)
        ] = 3

        padding_size = np.full(int((WINDOW_SIZE - 1) / 2), 0)
        padding_ranges = np.append(
            np.append(padding_size, orig_proc_ranges), padding_size
        )
        padding_ranges = (
            np.convolve(padding_ranges, np.ones(WINDOW_SIZE), "valid") / WINDOW_SIZE
        )

        angle_ranges = np.arange(data.angle_min, data.angle_max, data.angle_increment)

        self.min_angle_index = int(
            (math.radians(-ANGLE_LIMIT) - data.angle_min) / data.angle_increment
        )
        max_angle_index = int(
            (math.radians(ANGLE_LIMIT) - data.angle_min) / data.angle_increment
        )

        final_ranges = padding_ranges[self.min_angle_index : max_angle_index]
        angle_ranges = angle_ranges[self.min_angle_index : max_angle_index]

        return final_ranges, angle_ranges

    def find_max_gap(self, free_space_ranges):
        start_idx = 0
        max_gap_num = 0
        curr_gap_num = 0
        curr_idx = 0

        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] > RANGE_THRESHOLD:
                curr_gap_num += 1
                if curr_gap_num == 1:
                    curr_idx = i
            else:
                if curr_gap_num > max_gap_num:
                    max_gap_num = curr_gap_num
                    start_idx = curr_idx
                curr_gap_num = 0

        if curr_gap_num > max_gap_num:
            max_gap_num = curr_gap_num
            start_idx = curr_idx
        return start_idx, start_idx + max_gap_num - 1

    def find_best_point(self, start_i, end_i):
        return int((start_i + end_i) / 2)

    def lidar_callback(self, data):
        proc_ranges, angle_ranges = self.preprocess_lidar(data)

        closest_index = np.argmin(proc_ranges)

        free_space_ranges = np.array(proc_ranges, copy=True)
        min_range = max(closest_index - BUBBLE_RADIUS, 0)
        max_range = min(closest_index + BUBBLE_RADIUS, len(free_space_ranges) - 1) + 1

        free_space_ranges[min_range:max_range] = 0

        start_idx, end_idx = self.find_max_gap(free_space_ranges)

        best_point_inx = self.find_best_point(start_idx, end_idx)

        steer_angle = angle_ranges[best_point_inx]

        velocity = 1
        for angle, vel in VELOCITY_LIMITS:
            if abs(steer_angle) <= math.radians(angle):
                velocity = vel
                break

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steer_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = ReactiveFollowGap()
    rospy.sleep(0.1)
    rospy.spin()


if __name__ == "__main__":
    main(sys.argv)
