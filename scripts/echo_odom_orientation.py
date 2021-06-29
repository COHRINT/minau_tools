#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf
import numpy as np
from std_msgs.msg import Float64

def callback(msg):
    global bias, pub

    quat = msg.pose.pose.orientation

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    print(np.degrees(yaw))

rospy.init_node("imu_echo")
rospy.Subscriber("odometry/filtered/odom", Odometry, callback)
rospy.spin()
