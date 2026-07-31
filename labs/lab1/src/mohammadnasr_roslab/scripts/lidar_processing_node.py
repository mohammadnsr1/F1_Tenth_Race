#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np

class LidarDataProcessor:
    def __init__(self):
        self.min_topic = rospy.Publisher('closest_point', Float64, queue_size=10)
        self.max_topic = rospy.Publisher('farthest_point', Float64, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback)
    
    def callback(self, data):
        ranges = [r for r in data.ranges if r != float('inf') and not np.isnan(r)]
        min_range = Float64(min(ranges))
        max_range = Float64(max(ranges))
        self.min_topic.publish(min_range)
        self.max_topic.publish(max_range)

if __name__ == '__main__':
    rospy.init_node('lidar_processor', anonymous=True)
    LidarDataProcessor()
    rospy.spin()

