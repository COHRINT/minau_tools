#!/usr/bin/env python

import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu

def callback(imu):
    pose = imu
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    print('RPY: ({:0.2f}, {:0.2f}, {:0.2f})'.format(np.degrees(roll), np.degrees(pitch), np.degrees(yaw)))


actor = "bluerov2_3"
rospy.init_node("imu_echo")
rospy.Subscriber("{}/imu/data".format(actor), Imu, callback)
rospy.spin()
