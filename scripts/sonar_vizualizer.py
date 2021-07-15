#!/usr/bin/env python
import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from minau.msg import SonarTargetList, SonarTarget
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
		elif model == "prev":
			self.model = self.previous_scan_models
		elif model == "outlier":
			self.prev_10 = [None for i in range(10)]
			self.model = self.outlier_model
		self.range = None

		self.cutoff = 50
		self.image_pub = rospy.Publisher("ping360_node/sonar/detection_image_"+model,Image,queue_size=10)
		self.detection_pub = rospy.Publisher("sonar_processing/target_list", SonarTargetList, queue_size=10)
		
		self.previous_2 = [None,None]

		rospy.Subscriber("ping360_node/sonar/data",SonarEcho,self.data_callback)
		self.raw_image = np.zeros((500,500,3),np.uint8)
		self.image = np.zeros((500, 500, 3), np.uint8)
		self.bridge = CvBridge()



	def data_callback(self,msg):
		self.range = float(msg.range)
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
			samples_per_meter = int(len(data) / float(self.range))
			self.detection_dict[int(angle)] = np.argmax(data[samples_per_meter:]) + samples_per_meter
			rad_angle = float(angle + 200) * (np.pi/200.)
			while rad_angle > np.pi:
				rad_angle -= 2*np.pi
			while rad_angle < -np.pi:
				rad_angle += 2*np.pi

			detection_range = self.detection_dict[int(angle)] /float(samples_per_meter)


			target = SonarTarget("detection", rad_angle, 0.1, 0, 0.1, detection_range, 0.1, False, 
				SonarTarget().TARGET_TYPE_UNKNOWN, 0)
			header = msg.header
			header.frame_id = "base_link"
			stl = SonarTargetList(header, [target])
			self.detection_pub.publish(stl)
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
		THRESHOLD = 100
		DIST_DECREASE_PER_M = 7
		blend_data = [0.0] * (len(data)-self.cutoff)
		for i in range(len(data)-self.cutoff):
			blend_data[i] = data[i+self.cutoff] + data[i+self.cutoff -1] + data[i+self.cutoff-2] + data[i+self.cutoff-3] + data[i+self.cutoff-4]
			blend_data[i] = blend_data[i]/5
			idx = i + self.cutoff
			dist = (idx * float(self.range))/len(data)
			if blend_data[i] >= THRESHOLD-DIST_DECREASE_PER_M*dist:
				return True
		return False
	
	def previous_scan_models(self,data):
		data_np = np.array(data)
		if self.previous_2[0] is not None and self.previous_2[1] is not None:
			combined_data = (data_np + self.previous_2[0] + self.previous_2[1])/3.0
			self.previous_2[0],self.previous_2[1] = self.previous_2[1], data_np
			return self.blend_model(combined_data)
		else:
			self.previous_2[0],self.previous_2[1] = self.previous_2[1], data_np
			return False

	
	def outlier_model(self,data):
		for i in range(10):
			if self.prev_10[i] is None:
				self.prev_10[:9] = self.prev_10[1:]
				self.prev_10[9] = data
				return False
		samples_per_meter = int(len(data) / 10.0)
		count = 0
		for i in range(len(data)-samples_per_meter):
			first = False
			detect = True
			for j in range(10):
				if self.prev_10[j][i] > data[i]:
					if not first:
						first = True
					else:
						detect = False
						break
			if detect:
				count += 1
				if count == 3:
					return True
			else:
				count = 0
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











