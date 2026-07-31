#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from mohammadnasr_roslab.msg import scan_range
import numpy as np

class ProblemC:
    def __init__(self):
        self.min_max_range_topic = rospy.Publisher('scan_range', scan_range, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback)
    
    def callback(self, data):
        ranges = [r for r in data.ranges if r != float('inf') and not np.isnan(r)]
	min_range = min(ranges)
	max_range = max(ranges)
	msg = scan_range()
	msg.min_range = min_range
	msg.max_range = max_range
        self.min_max_range_topic.publish(msg)

if __name__ == '__main__':
    rospy.init_node('problem_C', anonymous=True)
    ProblemC()
    rospy.spin()
