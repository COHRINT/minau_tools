#!/usr/bin/env python

"""
This script converts the beacon measurement to a pose measurement that can be consumed by robot localization
"""

import rospy
from etddf_minau.msg import Measurement, MeasurementPackage
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

MEASUREMENT_COVARIANCE = 4

rospy.init_node("beacon_to_pose")
pub = rospy.Publisher("beacon_pose", PoseWithCovarianceStamped, queue_size=1)

my_name = "dory"#rospy.get_namespace().replace("/","")

def callback(msg):
    global pub, my_name, MEASUREMENT_COVARIANCE

    azimuth, _range = None, None
    global_pose = [0,0,0,0] # x,y,z, yaw (degrees)
    for meas in msg.measurements:
        if meas.measured_asset != my_name:
            continue
        elif meas.src_asset != "topside":
            rospy.logerr("Expected src asset to be topside!! was: {}".format(meas.src_asset))
            break


        if meas.meas_type == "modem_elevation":
            pass
        elif "azimuth" in meas.meas_type:
            global_pose = list(meas.global_pose)
            azimuth = (meas.data * np.pi) / 180
        elif "range" in meas.meas_type:
            _range = meas.data
    if azimuth is None:
        rospy.logerr("Azimuth data missing")
        return
    if _range is None:
        rospy.logerr("_range data missing")
        return

    # Linearize the measurement
    yaw_global_rad = np.radians( global_pose[3] )

    position_x = _range * np.cos( azimuth ) + global_pose[0]
    position_y = _range * np.sin( azimuth ) + global_pose[1]

    print("Range: {}, Az: {}--> X: {}, Y: {}".format(_range, azimuth, position_x, position_y))
    
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = rospy.get_rostime()
    pose_msg.header.frame_id = "odom"

    pose_msg.pose.pose.position.x = position_x
    pose_msg.pose.pose.position.y = position_y

    cov = np.diag([MEASUREMENT_COVARIANCE, MEASUREMENT_COVARIANCE,-1,-1,-1,-1])
    pose_msg.pose.covariance = list(cov.flatten())
    pub.publish(pose_msg)

rospy.Subscriber("etddf/packages_in", MeasurementPackage, callback)
rospy.spin()