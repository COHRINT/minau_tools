#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3Stamped, Vector3
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

rospy.init_node("dvl")
pub = rospy.Publisher("dvl", Vector3Stamped, queue_size=2)

rate = rospy.Rate(10)
def callback(msg):
    global pub

    #Find rotation matrix from quaternion
    q = msg.pose.pose.orientation
    r = R.from_quat([q.x, q.y, q.z, q.w]).as_dcm().transpose()

    v = msg.twist.twist.linear
    v3 = np.array([v.x,v.y,v.z]).transpose()

    #Find velocity in body frame
    dvl_v = r.dot(v3)

    #Add Noise
    dvl_x = dvl_v[0] + np.random.normal(0, scale=0.005)
    dvl_y = dvl_v[1] + np.random.normal(0, scale=0.005)
    dvl_z = dvl_v[2] + np.random.normal(0, scale=0.005)


    new_vel = Vector3(dvl_x, dvl_y, dvl_z)
    new_msg = Vector3Stamped()
    new_msg.vector = new_vel
    #get header set up
    new_msg.header.stamp = rospy.get_rostime()
    new_msg.header.frame_id = rospy.get_namespace() + 'base_link'

    pub.publish(new_msg)
    rate.sleep()

rospy.Subscriber("pose_gt", Odometry, callback, queue_size=1)
rospy.spin()