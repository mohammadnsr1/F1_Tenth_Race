#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float64

# PID CONTROL PARAMS
kp = 0.55
kd = 0.2
ki = 0.0001
servo_offset = 0.0
prev_error = 0.0
error = 0.0
integral = 0.0

# WALL FOLLOW PARAMS
ANGLE_RANGE = 270
DESIRED_DISTANCE_RIGHT = 0.9
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters
ld = 0.9

class WallFollow:
    """ Implement Wall Follow on the car """
    def __init__(self):
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

        self.distance_pub = rospy.Publisher("real_distance", Float64, queue_size=10)
        self.refer_dis_pub = rospy.Publisher("ref_distance", Float64, queue_size=10)

    def getRange(self, data, angle):
        """ Returns the distance to an object at a specific angle using LIDAR data. """
        self.ranges = np.array(data.ranges)
        self.arc_angle = math.radians(angle)
        self.trans_angle = math.radians(angle + 90)

        if self.arc_angle >= math.radians(-45) and self.arc_angle <= math.radians(225):
            self.arc_angle_index = int((self.trans_angle / math.radians(360)) * len(self.ranges))
            if not np.isnan(self.ranges[self.arc_angle_index]) and not np.isinf(self.ranges[self.arc_angle_index]):
                return self.ranges[self.arc_angle_index]

    def pid_control(self, error, velocity):
        """ Calculates and applies PID control based on the error. """
        global integral, prev_error, kp, ki, kd
        integral += error
        derivative = error - prev_error
        angle = -(kp * error + ki * integral + kd * derivative)
        prev_error = error
        rospy.loginfo('Angle: {}'.format(math.degrees(angle)))

        if 0 <= abs(angle) <= math.radians(10):
            velocity = 1.2
        elif abs(angle) <= math.radians(20):
            velocity = 0.5
        else:
            velocity = 0.5

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        """ Follows the left wall based on LIDAR data. """
        a = self.getRange(data, 150)
        b = self.getRange(data, 180)

        theta = math.radians(45)
        alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
        dt = b * math.cos(alpha)
        dt1 = dt + ld * math.sin(alpha)

        error = leftDist - dt1
        self.distance_pub.publish(dt)
        self.refer_dis_pub.publish(leftDist)
        return error

    def lidar_callback(self, data):
        """ Callback function for LIDAR data. """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)

