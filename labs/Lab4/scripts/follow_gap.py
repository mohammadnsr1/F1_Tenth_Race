#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class reactive_follow_gap:
    def __init__(self):
        # Topics & Subscriptions, Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.bubble_radius = 1  # safety bubble radius in meters

    def preprocess_lidar(self, ranges):
        # Rejecting high values (e.g., > 3)
        ranges[ranges > 10] = 0
	#ranges[ranges < 0 ] = 0
        return ranges

    def find_max_gap(self, free_space_ranges):
	    """Return the start index and end index of the max gap in free_space_ranges."""
	    # Find indices where obstacles (zero or less values) end or start
	    changes = np.diff(np.concatenate(([0], free_space_ranges > 0, [0])))
	    gap_starts = np.where(changes == 1)[0]
	    gap_ends = np.where(changes == -1)[0] - 1

	    # Calculate lengths of each gap
	    if len(gap_starts) == 0 or len(gap_ends) == 0:
		return 0, 0  # No gaps found

	    gap_lengths = gap_ends - gap_starts + 1
	    max_gap_idx = np.argmax(gap_lengths)  # Find the index of the maximum gap length

	    # Return the start and end indices of the maximum gap
	    return gap_starts[max_gap_idx], gap_ends[max_gap_idx]

    def find_best_point(self, start_i, end_i, ranges):
	"""Return index of best point in range."""
	# Naive approach: Find the furthest point within the max gap
	max_range_idx = np.argmax(ranges[start_i:end_i+1]) + start_i
	return max_range_idx

    def lidar_callback(self, data):
	"""Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message."""
	ranges = np.array(data.ranges)
	proc_ranges = self.preprocess_lidar(ranges)

	# Find the index of the closest point in the processed ranges
	bubble_idx = np.argmin(proc_ranges[proc_ranges>0])
	theta = self.bubble_radius/np.mean(proc_ranges)
	# Calculate the number of points that fit within the bubble_radius
	angle_increment = data.angle_increment
	num_points_in_bubble = int(theta / angle_increment)

	# Set all points within the bubble radius to zero
	start_idx = max(0, bubble_idx - num_points_in_bubble)
	end_idx = min(len(proc_ranges), bubble_idx + num_points_in_bubble)
	proc_ranges[start_idx:end_idx + 1] = 0

	# Find the maximum length gap
	start_idx, end_idx = self.find_max_gap(proc_ranges)

	# Find the best point in the gap
	best_point_idx = self.find_best_point(start_idx, end_idx, proc_ranges)

	# Calculate the angle to the best point
	angle = data.angle_min + best_point_idx * angle_increment

	# Publish Drive message
	drive_msg = AckermannDriveStamped()
	drive_msg.header.stamp = rospy.Time.now()
	drive_msg.drive.steering_angle = angle
	drive_msg.drive.speed = 0.8  # Set a constant speed or adjust based on context
	self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

