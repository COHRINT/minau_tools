#!/usr/bin/env python
import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from math import cos, pi, sin
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from minau.msg import SonarTargetList, SonarTarget
import cv2
import copy

GREEN = (0,255,0)
IMAGE_WIDTH = 500
IMAGE_HEIGHT = 500


def grad_to_rad(grads):
    return 2 * pi * grads / 400


class Visualizer:
	def __init__(self):
		self.image_pub = rospy.Publisher("ping360_node/sonar/image",Image,queue_size=10)
		self.range = None

		self.count = 0

		rospy.Subscriber("ping360_node/sonar/data",SonarEcho,self.data_callback)
		rospy.Subscriber("sonar_processing/target_list",SonarTargetList,self.detection_callback)
		self.image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), np.uint8)
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
												][2] = 255-pointColor
					self.cache[int(center[0] + x)][int(center[1] + y)
												][0] = pointColor
					self.cache[int(center[0] + x)][int(center[1] + y)
												][2] = 255-pointColor
					
		except IndexError:
			rospy.logwarn(
				"IndexError: data response was empty, skipping this iteration..")
			

		self.publishImage()

	def publishImage(self):
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "rgb8"))



if __name__ == "__main__":
	rospy.init_node("sonar_visualizer")
	Visualizer()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()