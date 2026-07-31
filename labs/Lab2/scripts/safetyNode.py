#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np


class safetyNode:
    def __init__(self,name):
	rospy.init_node(name, anonymous=True)
        self.brake = rospy.Publisher('/brake', AckermannDriveStamped, queue_size=10)
        self.brake_bool = rospy.Publisher('/brake_bool', Bool, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.x_velocity = 0
    
    def scan_callback(self, data):
	num_ranges = len(data.ranges)
        mid_index = num_ranges//2 - 1
	if self.x_velocity > 0:
	   start_index = mid_index - num_ranges//12
	   end_index = mid_index + num_ranges//12
	   self.TTC_cal(start_index,end_index,data,num_ranges)
	else:
	   rear_start_index_1 = num_ranges - num_ranges // 12
	   rear_end_index_1 = num_ranges
	   self.TTC_cal(rear_start_index_1,rear_end_index_1,data,num_ranges)
	   rear_start_index_2 = 0
           rear_end_index_2 = num_ranges // 12
   	   self.TTC_cal(rear_start_index_2,rear_end_index_2,data,num_ranges)

    def odom_callback(self,data):
	self.x_velocity = data.twist.twist.linear.x
	
    def Ebrake(self):
	rospy.loginfo('TTC:%s',self.beam_ttc)
	bool_msg = Bool(True)
	drive_msg = AckermannDriveStamped()
	drive_msg.drive.speed = 0
	self.brake_bool.publish(bool_msg)
	self.brake.publish(drive_msg)

    def TTC_cal(self,start_index,end_index,data,num_ranges):

	for i in range(start_index,end_index):
            distance = data.ranges[i % num_ranges]
	    if distance == float('inf') or np.isnan(distance):
	       continue
            angle = data.angle_min + (i % num_ranges) * data.angle_increment
            r_dot = max(self.x_velocity * np.cos(angle), 0)
	    if r_dot > 0:
	       self.beam_ttc = distance/ r_dot
	       if self.beam_ttc  < 0.4:
	          self.Ebrake()


if __name__ == '__main__':
    safetyNode('AEB')
    rospy.spin()
