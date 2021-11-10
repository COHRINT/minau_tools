#!/usr/bin/env python
import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from math import ceil, pi
import sensor_msgs.msg._Image
from darknet_ros_msgs.msg import BoundingBoxes
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
from minau.msg import SonarTargetList, SonarTarget


NUM_GRAD = 40
SCAN_IMAGE_ROWS = 3

IMAGE_WIDTH = 500
IMAGE_HEIGHT = 500

def grad_to_rad(grads):
    return 2 * pi * grads / 400



class Visualizer:
	def __init__(self,num_grads,scan_image_rows):
		self.image_pub = rospy.Publisher("ping360_node/sonar/cropped_image",sensor_msgs.msg.Image,queue_size=10)

		rospy.Subscriber("ping360_node/sonar/data",SonarEcho,self.data_callback)

		self.num_gradians = num_grads
		self.image = None
		self.bridge = CvBridge()
		self.ANGLE_IDX = 1
		self.DATA_IDX =0
		self.image_count = 1
		self.SCAN_IMG_ROWS = scan_image_rows

		self.first_angle = None
		self.step = None
		self.angle_queue = []


	def add_200_gradians(self,angle):
		new_angle = angle + 200

		while new_angle > 200:
			new_angle -= 400
		
		while new_angle <= -200:
			new_angle += 400

		return new_angle

	def data_callback(self,msg):
		#Set up data from bytes it comes in as
		data = [0]*len(msg.intensities)
		for i in range(len(data)):
			data[i] = ord(msg.intensities[i])

		converted_angle = self.add_200_gradians(msg.angle)

		if self.first_angle is None:	#If it is first scan save that angle and start the queue
			self.image = np.zeros((self.num_gradians*self.SCAN_IMG_ROWS, len(data), 3), np.uint8)
			self.first_angle = converted_angle
			self.angle_queue.append([data,converted_angle])
			return	

		if self.first_angle is not None and self.step is None:	#If second angle determine step and initialize rest of queue
			if self.first_angle > converted_angle:
				self.step =  int(self.first_angle - converted_angle)
				self.angle_queue.append([data,converted_angle])

				ratio = int(ceil(self.num_gradians/float(self.step)))			#Keep 40 gradians in queue
				empty_data = [0]*len(msg.intensities)


				for i in range(1,ratio-1):	#Insert empty data into the queue
					self.angle_queue.insert(0,[empty_data, self.first_angle + self.step * i])
				return

			else:	#Reset angle in case of edge cases
				self.first_angle = converted_angle
				self.angle_queue = [[data,converted_angle]]
				return
		
		# Check for jumps
		if converted_angle > self.angle_queue[-1][1]: #If it jumps plot data and restart queue
			self.plot_publish_data()				

			self.angle_queue = self.angle_queue[1:]
			self.angle_queue.append([data,converted_angle])

			empty_data = [0]*len(msg.intensities)

			for i in range(1,len(self.angle_queue)):
				self.angle_queue[-i-1] = [empty_data,converted_angle+(i*self.step)]


			self.first_angle = converted_angle
			return

		# Plot and publish data
		if self.first_angle - converted_angle >= self.num_gradians:
			self.plot_publish_data()
			self.first_angle -= int(self.num_gradians)
		
		
		# Pop off from queue and push new data on
		self.angle_queue = self.angle_queue[1:]
		self.angle_queue.append([data,converted_angle])
		# print(len(self.angle_queue))
		
	def plot_publish_data(self):	
		# self.image = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), np.uint8)
		for j in range(len(self.angle_queue)):
			data = self.angle_queue[j][self.DATA_IDX]

			for i in range(len(data)):
				pointColor = data[i]
				for s in range(self.step):
					for k in range(self.SCAN_IMG_ROWS):
						self.image[self.SCAN_IMG_ROWS*self.step*j + self.SCAN_IMG_ROWS*s + k][i][0] = pointColor
						self.image[self.SCAN_IMG_ROWS*self.step*j + self.SCAN_IMG_ROWS*s + k][i][1] = pointColor
		middle_angle = int(self.angle_queue[0][self.ANGLE_IDX] - (self.num_gradians/2.0))
		self.publishImage(middle_angle)
		return

	def publishImage(self,middle_angle):
		# print("publishing")
		# im = Image.fromarray(self.image)
		# im.save("your_file.png")
		ros_img = self.bridge.cv2_to_imgmsg(self.image, "rgb8")
		ros_img.header.stamp = rospy.get_rostime()
		ros_img.header.frame_id = str(middle_angle)
		self.image_pub.publish(ros_img)
		# print(self.image_count)
		self.image_count += 1


class DetectionRePublisher:
	def __init__(self,num_grads,scan_image_rows):
		self.data_sub = rospy.Subscriber("ping360_node/sonar/data",SonarEcho,self.data_callback)
		self.range = None
		self.image_width = None
		self.num_grads = num_grads
		self.image_height = num_grads * scan_image_rows


		rospy.Subscriber("darknet_ros/bounding_boxes",BoundingBoxes,self.detection_callback)

		self.detection_pub = rospy.Publisher("sonar_processing/target_list",SonarTargetList,queue_size=10)
		self.curr_seq = -1
	
	def data_callback(self,msg):
		# Listen to one message to get range then unregister subscriber
		self.range = float(msg.range)
		self.image_width = len(msg.intensities)
		self.data_sub.unregister()

	def detection_callback(self,msg):
		if not msg.image_header.seq > self.curr_seq:
			return
		
		self.curr_seq = msg.image_header.seq
		print("Detection: " + str(self.curr_seq))
		frame_grad_angle = float(msg.image_header.frame_id)

		for bounding_box in msg.bounding_boxes:

			detection_x = ((bounding_box.xmax + bounding_box.xmin)/2.0)
			detection_y = ((bounding_box.ymax + bounding_box.ymin)/2.0) - 10

			detection_range = detection_x * self.range / self.image_width

			image_angle = detection_y * self.num_grads / self.image_height
			total_grad_angle = frame_grad_angle + 20 - image_angle
			rad_angle = grad_to_rad(total_grad_angle)

			while rad_angle > np.pi:
				rad_angle -= 2*np.pi

			while rad_angle < -np.pi:
				rad_angle += 2*np.pi

			target = SonarTarget("Detection", rad_angle, 0.1, 0, 0.1, detection_range, 0.1, False, 
				SonarTarget().TARGET_TYPE_UNKNOWN, SonarTarget().UUV_CLASS_UNKNOWN)
			header = msg.image_header
			header.frame_id = "base_link"
			stl = SonarTargetList(header, [target])
			self.detection_pub.publish(stl)
		

	




if __name__ == "__main__":
	rospy.init_node("cropped_sonar_visualizer")
	Visualizer(NUM_GRAD, 1)
	# DetectionRePublisher(NUM_GRAD, SCAN_IMAGE_ROWS)
	rospy.spin()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()
