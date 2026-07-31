#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Bool
from ackermann_msgs.msg import AckermannDriveStamped
import math

class SafetyNode:
    def __init__(self):
        rospy.init_node('closest_farthest_point_finder')
        
        self.ttc_threshold_publisher = rospy.Publisher('/ttc_publisher', Float64, queue_size=10)
        self.min_ttc_publisher = rospy.Publisher('/min_ttc', Float64, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.ttc_array = []
        self.speed = -0.8  # Negative speed for reverse motion
        self.brake_publisher = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/nav_0', AckermannDriveStamped, queue_size=1)
        self.brake_bool_publisher = rospy.Publisher("brake_bool", Bool, queue_size=1)

        self.ttc_threshold = 0.9  # Time-to-collision threshold in seconds

    def scan_callback(self, msg):
        # By default, set the vehicle to move backward
        msg_drive = AckermannDriveStamped()
        msg_drive.drive.speed = self.speed
        self.brake_publisher.publish(msg_drive)

        brake = False
        for i, range_val in enumerate(msg.ranges):
            if range_val != float('inf') and range_val > 0: 
                v_i = self.speed * math.cos(msg.angle_min + i * msg.angle_increment)
                if v_i < 0:  # Check for negative velocities since moving backward
                    ttc = range_val / -v_i  # Calculate TTC with positive speed value
                    self.ttc_array.append(ttc)
                    if ttc < self.ttc_threshold:
                        brake = True
                        brake_msg = AckermannDriveStamped()
                        brake_msg.header.stamp = rospy.Time.now()
                        brake_msg.drive.speed = 0  # Set speed to 0 to brake
                        self.brake_publisher.publish(brake_msg)
                        break
        self.min_ttc_publisher.publish(min(self.ttc_array))
        self.ttc_threshold_publisher.publish(self.ttc_threshold)
        brake_bool_msg = Bool()
        brake_bool_msg.data = brake
        self.brake_bool_publisher.publish(brake_bool_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = SafetyNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

