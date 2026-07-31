#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def commander():
    pub = rospy.Publisher('turtle1/cmd_vel',Twist , queue_size=10)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(1000) # 1000hz
    while not rospy.is_shutdown():
        vel_cmd = Twist()
	vel_cmd.linear.x = 2.0
	vel_cmd.linear.y = -2.0
	vel_cmd.angular.z = 2.0
        rospy.loginfo(vel_cmd)
        pub.publish(vel_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass
