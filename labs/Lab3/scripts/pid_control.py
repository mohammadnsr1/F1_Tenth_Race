#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 0.55
kd = 0.2
ki = 0.0001
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 360 
DESIRED_DISTANCE_RIGHT = 0.9 
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2 # meters per second
CAR_LENGTH = 0.5 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Follow on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
	index = angle * 3
        dist = data.ranges[index]
        if np.isnan(dist) or np.isinf(dist):
            return 0.0
        else:
            return dist


    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
	angle = 0.0
        integral += error
        derivative = error - prev_error
        angle = -(kp*error + ki*integral + kd*derivative)
        prev_error = error
 	rospy.loginfo('Angle: {}'.format(math.degrees(angle)))

	if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 1.5
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 1
        else:
            velocity = 0.5
	
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm
        a = self.getRange(data,210)
        b = self.getRange(data,270)
        alpha = math.atan((a * math.cos(math.radians(60)) - b) / (a * math.sin(math.radians(60))))
	rospy.loginfo('Alpha: {}'.format(alpha))
        D_t = b*math.cos(alpha)
        D_tplus1 = D_t + math.sin(alpha)*CAR_LENGTH
	error = (DESIRED_DISTANCE_LEFT - D_tplus1)
	rospy.loginfo('Dt: {}'.format(D_tplus1))
	rospy.loginfo('Error: {}'.format(error))
        return error

    def lidar_callback(self, data):
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
