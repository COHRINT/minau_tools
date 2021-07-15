#!/usr/bin/env python
"""
Test that all the sensors are publishing good Messages
"""

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from seatrac_driver.msg import DriverStatus


class SensorTester:
	def __init__(self):
		rospy.Subscriber("imu/data",Imu,self.imu_callback)
		rospy.Subscriber("dvl",TwistWithCovarianceStamped, self.dvl_callback)
		rospy.Subscriber("baro",PoseWithCovarianceStamped,self.baro_callback)
		rospy.Subscriber("driver_status",DriverStatus,self.modem_callback)

		self.num_done = 0
		self.imu_first = True
		self.dvl_first = True
		self.baro_first = True
		self.modem_first = True

	
	def imu_callback(self,msg):
		if self.imu_first:
			self.imu_first = False
			self.num_done += 1
			print("IMU Working: " + str(self.num_done) + "/4 total sensors working")
	
	def dvl_callback(self,msg):
		if self.dvl_first:
			vel = msg.twist.twist.linear
			if not np.isnan(vel.x) and not np.isnan(vel.y) and not np.isnan(vel.z):
				self.dvl_first = False
				self.num_done += 1
				print("DVL Working: " + str(self.num_done) + "/4 total sensors working")
			else:
				print("ERROR! DVL is giving NaNs")
	
	def baro_callback(self,msg):
		if self.baro_first:
			self.baro_first = False
			self.num_done += 1
			print("Baro Working: " + str(self.num_done) + "/4 total sensors working")

	def modem_callback(self,msg):
		if self.modem_first:
			if msg.parsed_packet_count > 1:
				self.modem_first = False
				self.num_done += 1
				print("Modem Working: " + str(self.num_done) + "/4 total sensors working")



if __name__ == "__main__":
	rospy.init_node("Sensor tester")
	st = SensorTester()

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		rate.sleep()



			
	
