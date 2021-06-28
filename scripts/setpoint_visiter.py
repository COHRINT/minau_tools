#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import os
import numpy as np
import argparse
import sys

parser = argparse.ArgumentParser(description='Setpoint visitor')
parser.add_argument("-a", "--actor", type=str, help="actor to control", required=True)
args = parser.parse_args()

actor = args.actor
print(actor)

pose = None

def callback(msg):
    global pose
    pose = msg.pose.pose


rospy.init_node("setpoint_visiter", anonymous=True)

if actor == "red_actor_5":
    rospy.Subscriber("{}/pose_gt".format(actor), Odometry, callback)
    rospy.wait_for_message("{}/pose_gt".format(actor), Odometry)
else:
    rospy.Subscriber("{}/strapdown/estimate".format(actor), Odometry, callback)
    rospy.wait_for_message("{}/strapdown/estimate".format(actor), Odometry)

print("rosservice call /{}/uuv_control/arm_control ".format(actor) + "{}")
os.system("rosservice call /{}/uuv_control/arm_control ".format(actor) + "{}")
print("armed...")

current_index = 0

# TODO if statement on waypoints
# setpoints = [ [6.0,0.0 ,1.0] ]
# [8, -3.2, 1.0], [13.5, -7.0, 1.0], [13.5, 4.0, 1.0]]

# TEST 1
# setpoints_dory = [[-4, 8, 1.0], [-2, 8, 1.0], [6, 9, 1.0], [4, -3, 1.0], [6, 9, 1.0], [6, -3, 1.0], [-5, -9, 1.0], [-4, 6, 1.0], [10, 0, 1.0], [-1, -4, 1.0], [6, -8, 1.0], [1, -8, 1.0], [-6, 2, 1.0], [-5, -1, 1.0], [-4, -4, 1.0], [-1, 7, 1.0], [-10, 9, 1.0], [0, -4, 1.0], [-3, 8, 1.0], [-5, -4, 1.0]]
# setpoints_guppy = [[-8, -4, 1.0], [2, -7, 1.0], [-3, 0, 1.0], [2, 8, 1.0], [1, -9, 1.0], [5, -8, 1.0], [10, -3, 1.0], [8, 4, 1.0], [5, -1, 1.0], [0, 1, 1.0], [-3, 2, 1.0], [-4, 1, 1.0], [-1, 0, 1.0], [6, 1, 1.0], [-6, -5, 1.0], [7, -2, 1.0], [6, -2, 1.0], [1, 10, 1.0], [-8, -6, 1.0], [-9, -5, 1.0]]
# setpoints_red = [[9, 7, 1.0], [-6, 8, 1.0], [1, -3, 1.0], [-10, -5, 1.0], [-10, 4, 1.0], [-10, -10, 1.0], [5, 5, 1.0], [3, -2, 1.0], [-2, 3, 1.0], [-7, 2, 1.0], [6, 4, 1.0], [-1, 9, 1.0], [5, 2, 1.0], [-1, -7, 1.0], [4, 3, 1.0], [-9, 8, 1.0], [-10, -5, 1.0], [-9, -2, 1.0], [-1, -1, 1.0], [-9, 4, 1.0]]

# TEST 2
# setpoints_dory = [[6, 2, 1], [-7, 0, 1], [1, 2, 1], [-4, -8, 1], [9, 0, 1], [2, -5, 1], [10, -3, 1], [8, 0, 1], [-6, -7, 1], [1, 10, 1], [-8, 10, 1], [-7, -3, 1], [-6, 8, 1], [7, -2, 1], [-9, -5, 1], [-5, 1, 1], [5, -8, 1], [-9, -6, 1], [-7, 7, 1], [6, 4, 1]]
# setpoints_guppy = [[-4, -2, 1], [-6, 4, 1], [9, -1, 1], [-6, 3, 1], [-8, -3, 1], [-3, -3, 1], [2, 4, 1], [7, 4, 1], [-4, -2, 1], [0, -2, 1], [3, 8, 1], [2, -3, 1], [4, -1, 1], [-2, 5, 1], [3, 3, 1], [-4, 4, 1], [7, -4, 1], [6, 3, 1], [-7, -8, 1], [8, 6, 1]]
# setpoints_red = [[6, 3, 1], [-6, -5, 1], [5, 0, 1], [-10, 4, 1], [-4, -6, 1], [9, 9, 1], [-1, -9, 1], [9, 2, 1], [4, 4, 1], [-6, -8, 1], [10, 7, 1], [-6, 5, 1], [10, 1, 1], [3, 2, 1], [-7, 8, 1], [6, 10, 1], [-3, 8, 1], [6, -4, 1], [1, 9, 1], [9, -7, 1]]

# TEST 3
# setpoints_dory = [[4, 6, 1], [10, 6, 1], [-6, -10, 1], [10, -8, 1], [7, -7, 1], [0, -8, 1], [3, 9, 1], [4, -6, 1], [-10, -9, 1], [-7, -6, 1], [-2, -5, 1], [2, 3, 1], [-2, 9, 1], [-6, 9, 1], [-6, -2, 1], [-3, -8, 1], [6, 1, 1], [9, -8, 1], [-4, 4, 1], [10, -5, 1]]
# setpoints_guppy = [[1, 10, 1], [-10, -1, 1], [-2, 3, 1], [-8, -1, 1], [3, -5, 1], [2, -6, 1], [-7, 8, 1], [4, 7, 1], [-2, -6, 1], [-1, -8, 1], [-8, -9, 1], [-5, -10, 1], [-3, -2, 1], [-2, -1, 1], [1, -8, 1], [9, -7, 1], [0, -1, 1], [1, 8, 1], [-5, -4, 1], [-3, -3, 1]]
# setpoints_red = [[-4, 6, 1], [9, 5, 1], [-10, 1, 1], [-5, 5, 1], [-2, 6, 1], [-10, -1, 1], [8, -6, 1], [-7, -7, 1], [-3, 7, 1], [9, -10, 1], [-2, -10, 1], [-5, 1, 1], [-2, -3, 1], [4, 2, 1], [-2, 1, 1], [2, 3, 1], [10, -3, 1], [6, 10, 1], [-8, 9, 1], [5, -1, 1]]

# TEST 4
setpoints_dory = [[1, -9, 1], [-1, 8, 1], [9, -8, 1], [2, 8, 1], [-6, -3, 1], [-9, -6, 1], [10, -5, 1], [4, 7, 1], [-6, -3, 1], [9, -10, 1], [-1, 6, 1], [3, 6, 1], [0, 0, 1], [0, 3, 1], [-8, -6, 1], [5, -5, 1], [-9, 2, 1], [7, -2, 1], [-3, 9, 1], [8, -1, 1]]
setpoints_guppy = [[9, -7, 1], [-7, 5, 1], [-7, 10, 1], [9, -5, 1], [10, -7, 1], [-8, -6, 1], [6, 4, 1], [0, 5, 1], [-5, 4, 1], [5, -1, 1], [2, -3, 1], [2, -9, 1], [5, 2, 1], [9, 8, 1], [1, 9, 1], [10, -4, 1], [1, -9, 1], [2, 9, 1], [8, -3, 1], [-5, 5, 1]]
setpoints_red = [[-7, -6, 1], [5, 6, 1], [-8, -9, 1], [9, 5, 1], [3, 3, 1], [5, 8, 1], [8, 8, 1], [-3, 8, 1], [3, 5, 1], [4, 0, 1], [7, -6, 1], [6, 3, 1], [-4, -1, 1], [-10, 7, 1], [-1, 9, 1], [-6, 10, 1], [-7, -2, 1], [4, 5, 1], [4, -10, 1], [-5, -10, 1]]

# TEST 5
# setpoints_dory = [[-7, -4, 1], [-5, -6, 1], [6, 6, 1], [-7, 10, 1], [-10, 6, 1], [-7, 6, 1], [4, -6, 1], [5, 1, 1], [1, 7, 1], [5, 0, 1], [4, -5, 1], [0, -7, 1], [0, 10, 1], [4, 5, 1], [-3, 2, 1], [5, 3, 1], [-4, 9, 1], [-8, 9, 1], [5, 4, 1], [-2, 0, 1]]
# setpoints_guppy = [[0, -3, 1], [-1, -7, 1], [4, 9, 1], [2, -3, 1], [-4, 10, 1], [-10, -9, 1], [-2, 5, 1], [9, 8, 1], [-7, 8, 1], [-5, 1, 1], [9, -9, 1], [2, -7, 1], [5, 3, 1], [10, 1, 1], [-8, 4, 1], [7, -3, 1], [6, -1, 1], [6, 2, 1], [-6, -9, 1], [-6, 2, 1]]
# setpoints_red = [[-7, 8, 1], [9, -6, 1], [-9, 0, 1], [4, -9, 1], [5, 8, 1], [7, 9, 1], [-10, 4, 1], [3, -7, 1], [-6, -7, 1], [2, 8, 1], [9, -10, 1], [1, -7, 1], [-10, 4, 1], [5, -9, 1], [5, -4, 1], [10, -2, 1], [-7, -1, 1], [8, 0, 1], [4, 1, 1], [6, 2, 1]]

if actor == "dory":
    setpoints = setpoints_dory
elif actor == "guppy":
    setpoints = setpoints_guppy
else:
    setpoints = setpoints_red

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


os.system("rosservice call /{}/uuv_control/set_heading_depth 'heading: ".format(actor) + str(heading_setpoint) + "\ndepth: "+str(depth_setpoint) + "'") 

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
        print("###### ADVANCING WAYPOINT ######")
        if current_index >= len(setpoints):
            os.system("rosservice call /{}/uuv_control/disarm_control ".format(actor) + "{}")
            print("REACHED FINAL WAYPOINT")
            break
        continue

    VEL = 0.2
    x_vel = VEL*np.cos(ori)
    y_vel = VEL*np.sin(ori)

    # transform ENU --> NED
    # x_vel, y_vel = y_vel, x_vel
    z_vel = 0.0
    # heading_setpoint = 90 - (ori * (180/np.pi))
    heading_setpoint = 0.0
    # heading_setpoint = ori * (180/np.pi)
    cmd = "rosservice call /"+ actor + "/uuv_control/set_heading_velocity '{heading: "
    os.system(cmd + str(heading_setpoint) + ", velocity: {x: "+str(x_vel) + ", y: " + str(y_vel) + ", z: " + str(z_vel) + "}}'") 
    # rospy.sleep(10)
    r.sleep()
os.system("rosservice call /bluerov2_5/uuv_control/set_heading_velocity '{heading: " + str(heading_setpoint) + ", velocity: {x: "+str(0.0) + ", y: " + str(0.0) + ", z: " + str(0.0) + "}}'") 
