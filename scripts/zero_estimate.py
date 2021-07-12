#!/usr/bin/env python

"""
Zeros the robot localization pose

This node is intended to be namespaced for the specific robot
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

rospy.init_node("zero_robot_localization_estimate")

pub = rospy.Publisher("set_pose", PoseWithCovarianceStamped, queue_size=10)

msg = PoseWithCovarianceStamped()
msg.header.stamp = rospy.get_rostime()
msg.header.frame_id = "odom"
cov = np.diag( [0.001,0.001, 0.1, 0.1, 0.1,0.1 ])
msg.pose.covariance = list(cov.flatten())
for i in range(5):
	pub.publish(msg)
	rospy.sleep(1)