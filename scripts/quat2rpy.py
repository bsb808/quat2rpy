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
from geometry_msgs.msg import Pose

class QuatToEuler():
    def __init__(self):
        self.euler_msg = Vector3Stamped()
        self.euler_deg_msg = Vector3Stamped()

        # Create subscribers and publishers.
        self.pub_imu_euler = rospy.Publisher("imu_euler", Vector3Stamped, queue_size = 1)
        self.pub_imu_euler_deg = rospy.Publisher("imu_euler_deg", Vector3Stamped,  queue_size = 1)
        self.pub_odom_euler = rospy.Publisher("odom_euler", Vector3Stamped, queue_size = 1)
        self.pub_odom_euler_deg = rospy.Publisher("odom_euler_deg", Vector3Stamped, queue_size = 1)
        self.pub_pose_euler = rospy.Publisher("pose_euler", Vector3Stamped, queue_size = 1)
        self.pub_pose_euler_deg = rospy.Publisher("pose_euler_deg", Vector3Stamped, queue_size = 1)

        
        sub_imu   = rospy.Subscriber("imu", Imu, self.imu_callback)
        sub_odom  = rospy.Subscriber("odom", Odometry, self.odom_callback)
        sub_pose  = rospy.Subscriber("pose", Pose, self.pose_callback)


    # Odometry callback function.
    def odom_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.fill_euler_msg(msg, r, p, y)
        self.pub_odom_euler.publish(self.euler_msg)
        self.pub_odom_euler_deg.publish(self.euler_deg_msg)


    def pose_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.fill_euler_msg(msg, r, p, y)
        self.pub_pose_euler.publish(self.euler_msg)
        self.pub_pose_euler_deg.publish(self.euler_deg_msg)


    # IMU callback function.
    def imu_callback(self, msg):
        # Convert quaternions to Euler angles.
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.fill_euler_msg(msg, r, p, y)
        self.pub_imu_euler.publish(self.euler_msg)
        self.pub_imu_euler_deg.publish(self.euler_deg_msg)

    # Fill in Euler angle message.
    def fill_euler_msg(self, msg, r, p, y):
        #self.euler_msg.header.stamp = msg.header.stamp
        self.euler_msg.vector.x = r
        self.euler_msg.vector.y = p
        self.euler_msg.vector.z = y
        #self.euler_deg_msg.header.stamp = msg.header.stamp
        self.euler_deg_msg.vector.x = r*180.0/pi
        self.euler_deg_msg.vector.y = p*180.0/pi
        self.euler_deg_msg.vector.z = y*180.0/pi

# Main function.    
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('quat_to_euler')

    # Class functions that do all the heavy lifting.
    quat_to_euler = QuatToEuler()

    # Main while loop.
    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
