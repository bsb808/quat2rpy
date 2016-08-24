#!/usr/bin/python
# -*- coding: utf-8 -*-

# Start up ROS pieces.
import roslib
import rospy
import tf

from math import pi

# ROS messages.
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

class QuatToEuler():
    def __init__(self):
        self.got_new_msg = False
        self.euler_msg = Vector3Stamped()
        self.euler_deg_msg = Vector3Stamped()

        # Create subscribers and publishers.
        sub_imu   = rospy.Subscriber("imu/filtered", Imu, self.imu_callback)
        #sub_odom  = rospy.Subscriber("odom", Odometry, self.odom_callback)
        pub_euler = rospy.Publisher("imu/euler", Vector3Stamped)
        pub_euler_deg = rospy.Publisher("imu/euler_deg", Vector3Stamped)

        # Main while loop.
        while not rospy.is_shutdown():
            # Publish new data if we got a new message.
            if self.got_new_msg:
                pub_euler.publish(self.euler_msg)
                pub_euler_deg.publish(self.euler_deg_msg)
                self.got_new_msg = False

    # Odometry callback function.
    def odom_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    # IMU callback function.
    def imu_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.fill_euler_msg(msg, r, p, y)

    # Fill in Euler angle message.
    def fill_euler_msg(self, msg, r, p, y):
        self.got_new_msg = True
        self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.vector.x = r
        self.euler_msg.vector.y = p
        self.euler_msg.vector.z = y
        self.euler_deg_msg.header.stamp = msg.header.stamp
        self.euler_deg_msg.vector.x = r*180.0/pi
        self.euler_deg_msg.vector.y = p*180.0/pi
        self.euler_deg_msg.vector.z = y*180.0/pi

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        quat_to_euler = QuatToEuler()
    except rospy.ROSInterruptException: pass
