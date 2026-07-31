#!/usr/bin/env python


import rospy
import numpy as np
from sensor_msgs.msg import LaserScan


def callback(data):
    lidar_ranges = np.array(data.ranges)
    filtered_ranges = lidar_ranges[np.isfinite(lidar_ranges)]
    rospy.loginfo(rospy.get_caller_id() + 'Filtered measurements:%s',filtered_ranges)
	
def scan_subscriber():
    rospy.init_node('scan_subscriber',anonymous=True)
    rospy.Subscriber('scan',LaserScan,callback)
    rospy.spin()

if __name__ == '__main__':
   scan_subscriber()

	

