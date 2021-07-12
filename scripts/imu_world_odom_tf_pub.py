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

imu_msg_temp = None
def callback(msg):
    global imu_msg_temp
    imu_msg_temp = msg

rospy.Subscriber("imu/data", Imu, callback)
rospy.wait_for_message("imu/data", Imu)
imu_msg = imu_msg_temp

br = tf.TransformBroadcaster()

r = rospy.Rate(20)
while not rospy.is_shutdown():
    orientation = [imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w]
    br.sendTransform(
        (0,0,0),
        orientation,
        rospy.Time.now(),
        "imu_world",
        "base_link")
    print("publishing---")
    r.sleep()