#!/usr/bin/env python


"""
This script is only necessary due to a bug in the robot localization stack where it won't accept the 
initial_pose rosparam.

This script reads in the initial_pose that robot localization should have and then calls the set_pose function 
& keeps publishing until robot_localization has accepted the new pose
"""

import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
import rospy
import tf

def _list2arr(l):
    return np.array([l]).reshape(-1,1)

rospy.init_node("robot_localization_pose_setter")

initial_pose = _list2arr([rospy.get_param("~x"), rospy.get_param("~y"), rospy.get_param("~z")])
initial_yaw = rospy.get_param("~starting_yaw")

pub = rospy.Publisher("set_pose", PoseWithCovarianceStamped, queue_size=10)

odom_topic = "odometry/filtered/odom"

new_pose = None
def callback(msg):
    global new_pose
    new_pose = _list2arr([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

rospy.Subscriber(odom_topic, Odometry, callback)

# First msg has published, robot localization has loaded
while not rospy.is_shutdown():
    rospy.wait_for_message(odom_topic, Odometry)
    if new_pose is not None:
        diff = np.linalg.norm(initial_pose - new_pose)
        if diff < 1: # Should be close to zero, but may have undergone a prediction step
            break
        else:   
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = rospy.get_rostime()
            msg.header.frame_id = "odom"
            msg.pose.pose.position.x = initial_pose[0,0]
            msg.pose.pose.position.y = initial_pose[1,0]
            msg.pose.pose.position.z = initial_pose[2,0]
            quat = tf.transformations.quaternion_from_euler(0, 0, initial_yaw)
            msg.pose.pose.orientation = Quaternion(*quat)
            new_cov = np.eye(6) * 0.01
            msg.pose.covariance = list(new_cov.flatten())
            pub.publish( msg )