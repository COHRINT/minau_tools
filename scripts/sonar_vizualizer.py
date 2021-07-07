#!/usr/bin/env python
import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from math import cos, pi, sin
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import argparse
import copy


def grad_to_rad(grads):
    return 2 * pi * grads / 400


class Visualizer:
	def __init__(self,model):
		self.detection_dict = {}
		for i in range(-196,204,4):
			self.detection_dict[i] = None


		if model == "blend":
			self.model = self.blend_model

		self.cutoff = 50
		self.image_pub = rospy.Publisher("ping360_node/sonar/detection_image_"+model,Image,queue_size=10)

		rospy.Subscriber("ping360_node/sonar/data",SonarEcho,self.data_callback)
		self.raw_image = np.zeros((500,500,3),np.uint8)
		self.image = np.zeros((500, 500, 3), np.uint8)
		self.bridge = CvBridge()



	def data_callback(self,msg):
		data = [0]*len(msg.intensities)
		for i in range(len(data)):
			data[i] = ord(msg.intensities[i])
		
		center = (250,250)
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
					self.raw_image[int(center[0] + x)][int(center[1] + y)
												][0] = pointColor
					self.raw_image[int(center[0] + x)][int(center[1] + y)
												][2] = 255-pointColor
					
		except IndexError:
			rospy.logwarn(
				"IndexError: data response was empty, skipping this iteration..")

		if self.model(data):
			print("detection")
			self.detection_dict[int(angle)] = np.argmax(data[self.cutoff:]) + self.cutoff
		else:
			self.detection_dict[int(angle)] = None

		self.image = copy.copy(self.raw_image)

		for i in range(-196,204,4):
			if self.detection_dict[i] is not None:
				radius = 4
				length = center[0] * self.detection_dict[i]/float(len(data))
				theta_new = grad_to_rad(i+2)
				x_offset = int(length * sin(theta_new))
				y_offset = int(length * cos(theta_new))
				circ_center = (center[0]+x_offset,center[1]+y_offset)
				

				cv2.circle(self.image,circ_center,radius,(0,255,0))


			

		self.publishImage()

	def publishImage(self):
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "rgb8"))


	def blend_model(self,data):
		THRESHOLD = 150
		blend_data = [0.0] * (len(data)-20)
		for i in range(len(data)-20):
			blend_data[i] = data[i+20] + data[i+19] + data[i+18] + data[i+17] + data[i+16]
			blend_data[i] = blend_data[i]/5
			if blend_data[i] >= THRESHOLD:
				return True
		return False
		



if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Sonar Detection Model')
	parser.add_argument("model", type=str, help="string representing specfic model in the file")
	args = parser.parse_args()
	model = args.model


	rospy.init_node("sonar_visualizer")
	Visualizer(model)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()











