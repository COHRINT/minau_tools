#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import os
import numpy as np

pose = None

def callback(msg):
    global pose
    pose = msg.pose.pose


rospy.init_node("setpoint_visiter")
rospy.Subscriber("strapdown/estimate", Odometry, callback)
rospy.wait_for_message("strapdown/estimate", Odometry)


os.system("rosservice call /bluerov2_5/uuv_control/arm_control {}")
print("armed...")

starting_x = 0
starting_y = 0
depth = 1.0

current_index = 0
# setpoints = [[starting_x, starting_y+5, depth], [starting_x+5, starting_y+5, depth],[starting_x+5, starting_y, depth], [starting_x, starting_y, depth]]
setpoints = [[starting_x+5, starting_y, depth], [starting_x+5, starting_y+5, depth],[starting_x, starting_y+5, depth], [starting_x, starting_y, depth]]

# New program
while 1:
    heading_setpoint = 0.0
    print(heading_setpoint)
    depth_setpoint = 1.0
    os.system("rosservice call /bluerov2_5/uuv_control/set_heading_depth 'heading: " + str(heading_setpoint) + "\ndepth: "+str(depth_setpoint) + "'") 
    rospy.sleep(15)
    heading_setpoint = 90.0
    print(heading_setpoint)
    os.system("rosservice call /bluerov2_5/uuv_control/set_heading_depth 'heading: " + str(heading_setpoint) + "\ndepth: "+str(depth_setpoint) + "'") 
    rospy.sleep(15)
    heading_setpoint = 180.0
    print(heading_setpoint)
    os.system("rosservice call /bluerov2_5/uuv_control/set_heading_depth 'heading: " + str(heading_setpoint) + "\ndepth: "+str(depth_setpoint) + "'") 
    rospy.sleep(15)
    heading_setpoint = 270.0
    print(heading_setpoint)
    os.system("rosservice call /bluerov2_5/uuv_control/set_heading_depth 'heading: " + str(heading_setpoint) + "\ndepth: "+str(depth_setpoint) + "'") 
    rospy.sleep(15)



print("diving...")
rospy.sleep(5)

r = rospy.Rate(1)

while not rospy.is_shutdown():
    x_target = setpoints[current_index][0]
    y_target = setpoints[current_index][1]
    x_current = pose.position.x
    y_current = pose.position.y
    diff_x = x_target - x_current
    diff_y = y_target - y_current
    ori = np.arctan2(diff_y, diff_x)
    print("Traversing to waypoint: {}, current position: {}".format(setpoints[current_index], [x_current, y_current]))

    if np.abs(diff_x) < 2 and np.abs(diff_y) < 2:
        current_index += 1
        print("ADVANCING WAYPOINT")
        if current_index == len(setpoints):
            break
        continue

    x_vel = 0.2*np.cos(ori)
    y_vel = 0.2*np.sin(ori)
    print([diff_x, diff_y])

    # transform ENU --> NED
    # x_vel, y_vel = y_vel, x_vel
    z_vel = 0.0
    # heading_setpoint = 90 - (ori * (180/np.pi))
    heading_setpoint = 0.0 #ori * (180/np.pi) + 90
    os.system("rosservice call /bluerov2_5/uuv_control/set_heading_velocity '{heading: " + str(heading_setpoint) + ", velocity: {x: "+str(y_vel) + ", y: " + str(x_vel) + ", z: " + str(z_vel) + "}}'") 
    # rospy.sleep(10)
    r.sleep()
os.system("rosservice call /bluerov2_5/uuv_control/set_heading_velocity '{heading: " + str(heading_setpoint) + ", velocity: {x: "+str(0.0) + ", y: " + str(0.0) + ", z: " + str(0.0) + "}}'") 