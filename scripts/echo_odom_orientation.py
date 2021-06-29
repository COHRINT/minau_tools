#!/usr/bin/env python

import rospy
import tf
import numpy as np
from nav_msgs.msg import Odometry

def callback(odom):
    pose = odom.pose.pose
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    print(np.degrees(yaw))

rospy.init_node("odom_ori_echo")
rospy.Subscriber("/odometry/filtered/odom", Odometry, callback)
rospy.spin()
