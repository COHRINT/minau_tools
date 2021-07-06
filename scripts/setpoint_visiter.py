#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import os
import numpy as np
import argparse
import sys

actor = "bluerov2_3"
print(actor)

#actor = args.actor
#print(actor)
actor = "bluerov2_3"
pose = None

def callback(msg):
    global pose
    pose = msg.pose.pose


rospy.init_node("setpoint_visiter", anonymous=True)

# if actor == "red_actor_5":
#     rospy.Subscriber("{}/pose_gt".format(actor), Odometry, callback)
#     rospy.wait_for_message("{}/pose_gt".format(actor), Odometry)
# else:
#     rospy.Subscriber("{}/strapdown/estimate".format(actor), Odometry, callback)
#     rospy.wait_for_message("{}/strapdown/estimate".format(actor), Odometry)
rospy.Subscriber("{}/odometry/filtered/odom".format(actor), Odometry, callback)
rospy.wait_for_message("{}/odometry/filtered/odom".format(actor), Odometry)

print("rosservice call /{}/uuv_control/arm_control ".format(actor) + "{}")
os.system("rosservice call /{}/uuv_control/arm_control ".format(actor) + "{}")
print("armed...")

current_index = 0

# ENU setpoints
# setpoints = [[0,3],[3,3],[0,3],[0,0]]
setpoints = [[5,0],[5,5],[0,5],[0,0]]

heading_setpoint = 90.0
depth_setpoint = 1.0

heading_setpoint = 90.0
depth_setpoint = 0.5

os.system("rosservice call /{}/uuv_control/set_heading_depth 'heading: ".format(actor) + str(heading_setpoint) + "\ndepth: "+str(depth_setpoint) + "'") 

print("diving...")
# sys.exit(0)

r = rospy.Rate(1)

while not rospy.is_shutdown():
    x_target = setpoints[current_index][0]
    y_target = setpoints[current_index][1]
    x_current = pose.position.x
    y_current = pose.position.y
    diff_x = x_target - x_current
    diff_y = y_target - y_current
    ori = np.arctan2(diff_y, diff_x)

    total_dist = np.linalg.norm([diff_x, diff_y])

    distance = 0.5
    if total_dist < distance:
        current_index += 1
        print("###### ADVANCING WAYPOINT ######")
        if current_index >= len(setpoints):
            os.system("rosservice call /{}/uuv_control/disarm_control ".format(actor) + "{}")
            print("REACHED FINAL WAYPOINT")
            break
        continue

    VEL = 0.1 if total_dist < distance*4 else 0.2
    x_vel = VEL*np.cos(ori)
    y_vel = VEL*np.sin(ori)

    # transform ENU --> NED
    x_vel, y_vel = y_vel, x_vel
    z_vel = 0.0
    cmd = "rosservice call /"+ actor + "/uuv_control/set_heading_velocity '{heading: "
    os.system(cmd + str(heading_setpoint) + ", velocity: {x: "+str(x_vel) + ", y: " + str(y_vel) + ", z: " + str(z_vel) + "}}'") 
    r.sleep()
