#!/usr/bin/env python
'''
	This allows you to click on points in rviz and have robot go there
'''

import rospy
from minau.msg import NavigateToActionGoal
from geometry_msgs.msg import PoseStamped


HEADING = 90.0

class Rviz_Waypoint:
	def __init__(self):

		self.waypoint_publisher = rospy.Publisher("uuv_control/navigate_to/goal", NavigateToActionGoal, queue_size=10)
		# this is the topic that rviz publishes to
		rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback=self.click_callback)

	def click_callback(self, msg):
		# publish waypoint with location that was clicked on rviz
		print("waypoint recieved")
		goal = NavigateToActionGoal()
		goal.header.stamp = rospy.get_rostime()
		goal.goal.arrival_radius = 0.0
		goal.goal.destination_position.x = msg.pose.position.y
		goal.goal.destination_position.y = msg.pose.position.x
		goal.goal.destination_position.z = 0.7
		goal.goal.release_at_target = False
		goal.goal.desired_heading = HEADING

		self.waypoint_publisher.publish(goal)

if __name__ == "__main__":
	rospy.init_node("rviz_waypoint")
	rw = Rviz_Waypoint()

	rospy.spin()