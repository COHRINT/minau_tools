#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np

rospy.init_node("baro")
pub = rospy.Publisher("baro", PoseWithCovarianceStamped, queue_size=2)
pub_mavros = rospy.Publisher("mavros/global_position/rel_alt", Float64, queue_size=2)

bias = rospy.get_param("mission_config/baro/bias")
var = rospy.get_param("mission_config/baro/var")

seq = 0
def callback(msg):
    global pub, seq, var, bias, pub_mavros

    true_bias = 0.3
    x = msg.pose.pose.position.z + np.random.normal(true_bias, scale=0.1)
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.get_rostime()
    msg.header.frame_id = "odom"
    msg.header.seq = seq

    msg.pose.pose.position.z = x - bias
    cov = np.diag([-1,-1,var,-1,-1,-1])
    msg.pose.covariance = list(cov.flatten())
    pub.publish(msg)
    seq += 1

    pub_mavros.publish(Float64(x))
    
rospy.Subscriber("pose_gt", Odometry, callback, queue_size=1)
rospy.spin()