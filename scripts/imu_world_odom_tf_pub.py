#!/usr/bin/env python
"""
This script sets the transform between imu_world and odom assuming
the odom orientation is 0 degrees ENU at runtime

Receives first imu message

"""
from sensor_msgs.msg import Imu
import rospy
import tf

rospy.init_node("imu_world_odom_tf_pub")

imu_msg = None
def callback(msg):
    global imu_msg
    imu_msg = msg

rospy.Subscriber("imu/data", Imu, callback)
rospy.wait_for_message("imu/data", Imu)

br = tf.TransformBroadcaster()

r = rospy.Rate(10)
while not rospy.is_shutdown():
    br.sendTransform(
        (0,0,0),
        imu_msg.orientation,
        rospy.Time.now(),
        "odom",
        "imu_world")
    r.sleep()