#!/usr/bin/env python
from wldvl import WlDVL
import rospy
from geometry_msgs.msg import Vector3Stamped
from cuprint.cuprint import CUPrint

dvl_pub = rospy.Publisher("dvl",Vector3Stamped,queue_size=5)
rospy.init_node('real_dvl')
cuprint = CUPrint(rospy.get_name())
cuprint("Loading")

dvl = WlDVL("/dev/dvl")
while not rospy.is_shutdown():
    report = dvl.read()
    if report is not None:
        if report['valid']:
            v = Vector3Stamped()
            v.header.stamp = rospy.get_rostime()
            v.header.frame_id = rospy.get_namespace() + 'base_link'
            v.vector.x = report['vx']
            v.vector.y = report['vy']
            v.vector.z = report['vz']
            dvl_pub.publish(v)
        else:
            cuprint("invalid read", warn=True)
