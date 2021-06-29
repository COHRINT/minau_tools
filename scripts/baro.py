#!/usr/bin/env python

import rospy
# from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np

rospy.init_node("baro")
pub = rospy.Publisher("baro", PoseWithCovarianceStamped, queue_size=2)

seq = 0
def callback(msg):
    global pub, seq
    x = msg.pose.pose.position.z + np.random.normal(0, scale=0.01)
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "odom"
    msg.header.seq = seq

    msg.pose.pose.position.z = x
    cov = np.diag([-1,-1,0.1,-1,-1,-1])
    msg.pose.covariance = list(cov.flatten())
    pub.publish(msg)
    seq += 1
    
rospy.Subscriber("pose_gt", Odometry, callback, queue_size=1)
rospy.spin()