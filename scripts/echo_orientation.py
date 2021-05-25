#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf
import numpy as np
from std_msgs.msg import Float64

def callback(imu_msg):
    global bias, pub

    roll, pitch, yaw = tf.transformations.euler_from_quaternion([imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w])
    print(np.degrees(yaw))

rospy.init_node("imu_echo")
rospy.Subscriber("mavros/imu/data/corrected", Imu, callback)
rospy.spin()