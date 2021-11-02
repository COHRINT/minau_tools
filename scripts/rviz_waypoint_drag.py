#!/usr/bin/env python

import rospy
from minau.msg import NavigateToActionGoal
from geometry_msgs.msg import PoseStamped



ASSET_NAME = "bluerov2_4"
HEADING = 0.0





class Rviz_Waypoint:
	def __init__(self):

		self.waypoint_publisher = rospy.Publisher("/{asset}/uuv_control/navigate_to/goal".format(asset = ASSET_NAME), NavigateToActionGoal, queue_size=10)

		rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback=self.click_callback)

	def click_callback(self, msg):

		# TODO: msg comes in in ENU, NEED TO CHANGE TO NED before we publish it
		print("waypoint recieved")
		goal = NavigateToActionGoal()
		goal.header.stamp = rospy.get_rostime()
		goal.goal.arrival_radius = 0.5
		goal.goal.destination_position.x = msg.pose.position.y
		goal.goal.destination_position.y = msg.pose.position.x
		goal.goal.destination_position.z = 0.5
		goal.goal.desired_heading = HEADING

		self.waypoint_publisher.publish(goal)

if __name__ == "__main__":
	rospy.init_node("rviz_waypoint")
	rw = Rviz_Waypoint()

	rospy.spin()