#!/usr/bin/env python
import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from math import cos, pi, sin
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from minau.msg import SonarTargetList, SonarTarget
from nav_msgs.msg import Odometry
import cv2
import copy
import tf

GREEN = (0,255,0)
IMAGE_WIDTH = 500
IMAGE_HEIGHT = 500


def grad_to_rad(grads):
    return 2 * pi * grads / 400


class Visualizer:
	def __init__(self):
		self.image_pub = rospy.Publisher("/ping360_node/sonar/image",Image,queue_size=10)
		self.range = None

		self.count = 0

		rospy.Subscriber("/ping360_node/sonar/data",SonarEcho,self.data_callback)
		rospy.Subscriber("/sonar_processing/target_list",SonarTargetList,self.detection_callback)
		self.image = np.ones((IMAGE_HEIGHT, IMAGE_WIDTH, 3), np.uint8) * 255
		self.cache = copy.deepcopy(self.image)
		self.bridge = CvBridge()

	def detection_callback(self,msg):
		self.image = copy.deepcopy(self.cache)
		self.count = 0
		for target in msg.targets:
			target_range = target.range_m
			target_angle = target.bearing_rad

			x = int((IMAGE_WIDTH / 2) - (target_range * sin(target_angle)) * ((IMAGE_WIDTH / 2) / self.range))
			y = int((IMAGE_HEIGHT / 2) - (target_range * cos(target_angle)) * ((IMAGE_WIDTH / 2) / self.range))
			print("x: " + str(x))
			print("y: " + str(y))
			print('')

			cv2.rectangle(self.image,(x - 10, y - 10), (x + 10, y + 10), GREEN)


	def data_callback(self,msg):
		if self.count > 60:
			self.image = copy.deepcopy(self.cache)
			self.count = 0
		self.count += 1

		self.range = msg.range
		data = [0]*len(msg.intensities)
		for i in range(len(data)):
			data[i] = ord(msg.intensities[i])
		
		center = (IMAGE_HEIGHT/2,IMAGE_WIDTH/2)
		linear_factor = float(len(data)) / float(center[0])
		step = 4
		angle = msg.angle
		try:
			for i in range(int(center[0])):
				if(i < center[0]):
					pointColor = data[int(i * linear_factor - 1)]
				else:
					pointColor = 0
				for k in np.linspace(0, step, 8 * step):
					theta = grad_to_rad(angle + k)
					# if convertToEnu:
					# 	theta = ned_to_enu(theta)
					x = float(i) * cos(theta)
					y = float(i) * sin(theta)
					self.image[int(center[0] + x)][int(center[1] + y)
												][0] = pointColor
					self.image[int(center[0] + x)][int(center[1] + y)
												][1] = pointColor
					self.image[int(center[0] + x)][int(center[1] + y)
												][2] = 0

					self.cache[int(center[0] + x)][int(center[1] + y)
												][0] = pointColor
					self.cache[int(center[0] + x)][int(center[1] + y)
												][1] = pointColor
					self.cache[int(center[0] + x)][int(center[1] + y)
												][2] = 0
					
		except IndexError:
			rospy.logwarn(
				"IndexError: data response was empty, skipping this iteration..")
			

		self.publishImage()

	def publishImage(self):
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "rgb8"))

class SonarScanRviz:
	def __init__(self):
		rospy.Subscriber("/ping360_node/sonar/data",SonarEcho,self.data_callback)
		rospy.Subscriber("odometry/filtered/odom", Odometry, self.pose_callback)
		self.sonar_odom_pub = rospy.Publisher("/sonar_odom", Odometry, queue_size=10)

		self.position = None
		self.curr_yaw = None

	def data_callback(self, msg):
		if self.position is None or self.curr_yaw is None:
			return

		rad_angle = grad_to_rad(msg.angle + 200)

		tot_angle = self.curr_yaw + rad_angle

		while tot_angle > np.pi:
			tot_angle -= 2 * np.pi
		
		while tot_angle < -np.pi:
			tot_angle += 2 * np.pi

		o = Odometry()
		o.header.frame_id = "odom"
		o.header.stamp = rospy.get_rostime()
		o.pose.pose.position = self.position

		quat_angle = tf.transformations.quaternion_from_euler(0, 0, tot_angle)
		o.pose.pose.orientation.x = quat_angle[0]
		o.pose.pose.orientation.y = quat_angle[1]
		o.pose.pose.orientation.z = quat_angle[2]
		o.pose.pose.orientation.w = quat_angle[3]

		self.sonar_odom_pub.publish(o)

	def pose_callback(self, msg):
		self.position = msg.pose.pose.position
		ang = msg.pose.pose.orientation
		(_, _, self.curr_yaw) = tf.transformations.euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])





if __name__ == "__main__":
	rospy.init_node("sonar_visualizer")
	Visualizer()
	SonarScanRviz()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()