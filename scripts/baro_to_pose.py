#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped

pub = None
seq = 0
depth_bias = None

def callback(float_msg):
    global pub, seq, depth_bias

    if depth_bias is None:
        depth_bias = float_msg.data

    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "odom"
    msg.header.seq = seq

    msg.pose.pose.position.z = float_msg.data - depth_bias
    cov = np.diag([-1,-1,0.1,-1,-1,-1])
    msg.pose.covariance = list(cov.flatten())
    pub.publish(msg)

    seq += 1


rospy.init_node("baro_repub")
rospy.Subscriber("mavros/global_position/rel_alt", Float64, callback)
pub = rospy.Publisher("baro", PoseWithCovarianceStamped, queue_size=10)
rospy.spin()
