#!/usr/bin/env python
from pyPS4Controller.controller import Controller
import numpy as np
import rospy
from minau.srv import ArmControl, SetHeadingVelocity, SetHeadingDepth
from geometry_msgs.msg import Vector3
from minau.msg import ControlStatus
from nav_msgs.msg import Odometry

def vel(heading,speed):
    heading_rad = heading*np.pi/180
    return Vector3(speed*np.cos(heading_rad),speed*np.sin(heading_rad),0)
class MyController(Controller):

    def __init__(self, **kwargs):
        # print(rospy.get_namespace())
        Controller.__init__(self, **kwargs)
        self.heading = 0.0
        self.depth = 0.0
        self.speed = 0.3
        self.dive_depth = 0.3
        rospy.wait_for_service('uuv_control/arm_control')
        arm_control = rospy.ServiceProxy('uuv_control/arm_control', ArmControl)
        resp1 = arm_control()
        # print(2)
        rospy.wait_for_service('uuv_control/set_heading_velocity')
        self.shv = rospy.ServiceProxy('uuv_control/set_heading_velocity', SetHeadingVelocity)
        # print(3)
        rospy.wait_for_service('uuv_control/set_heading_depth')
        self.shd = rospy.ServiceProxy('uuv_control/set_heading_depth', SetHeadingDepth)
        # print(4)
    #Dive    
    def on_square_press(self):
        self.shd(np.pi,self.dive_depth)
        self.depth = self.dive_depth
        print("Dive")
    def on_square_release(self):
        self.shd(self.heading,self.dive_depth)
        self.depth = self.dive_depth
        print("Dive")

    #Come to Surface
    def on_x_press(self):
        self.shd(self.heading,-1.0)
        self.depth = -1.0
        print("Rise")
    def on_x_release(self):
        self.shd(self.heading,-1.0)
        self.depth = -1.0
        print("Rise")

    def on_L3_up(self,value):
        print("Go Forward")
        v = vel(self.heading,self.speed)
        self.shv(self.heading,v)

    def on_L2_press(self,value):
        self.heading = (self.heading-2 +180)%(360) -180
        self.shd(self.heading,self.depth)
        print("Turn Left")
        print(self.heading)
    def on_L2_release(self):
        self.heading = (self.heading-2 +180)%(360) -180
        self.shd(self.heading,self.depth)
        print("Turn Left")
        print(self.heading)
    
    def on_R2_press(self,value):
        self.heading = (self.heading+2 +180)%(360) -180
        self.shd(self.heading,self.depth)
        print("Turn Right")
        print(self.heading)
    def on_R2_release(self):
        self.heading = (self.heading+2 +180)%(360) -180
        self.shd(self.heading,self.depth)
        print("Turn Right")
        print(self.heading)

    def on_R1_press(self):
        self.heading = (self.heading+90 +180)%(360) -180
        self.shd(self.heading,self.depth)
        print("Turn Right")
        print(self.heading)
    def on_R1_release(self):
        self.heading = (self.heading+90 +180)%(360) -180
        self.shd(self.heading,self.depth)
        print("Turn Right")
        print(self.heading)

    def on_L1_press(self):
        self.heading = (self.heading-90 +180)%(360) -180
        self.shd(self.heading,self.depth)
        print("Turn Right")
        print(self.heading)
    def on_L1_release(self):
        self.heading = (self.heading-90 +180)%(360) -180
        self.shd(self.heading,self.depth)
        print("Turn Right")
        print(self.heading)


    def on_up_arrow_press(self):
        print("Go Forward")
        v = vel(self.heading,self.speed)
        self.shv(self.heading,v)
    def on_up_arrow_release(self):
        print("Go Forward")
        v = vel(self.heading,self.speed)
        self.shv(self.heading,v)
    
    def on_down_arrow_press(self):
        print("Go Backward")
        dir1 = (self.heading+180 +180)%(360) -180
        v = vel(dir1,self.speed)
        self.shv(self.heading,v)
    def on_down_arrow_release(self):
        print("Go Backward")
        dir1 = (self.heading+180 +180)%(360) -180
        v = vel(dir1,self.speed)
        self.shv(self.heading,v)

    def on_right_arrow_press(self):
        print("Strafe Right")
        dir1 = (self.heading+90 +180)%(360) -180
        v = vel(dir1,self.speed)
        self.shv(self.heading,v)
    def on_right_arrow_release(self):
        print("Strafe Right")
        dir1 = (self.heading+90 +180)%(360) -180
        v = vel(dir1,self.speed)
        self.shv(self.heading,v)

    def on_left_arrow_press(self):
        print("Strafe Left")
        dir1 = (self.heading-90 +180)%(360) -180
        v = vel(dir1,self.speed)
        self.shv(self.heading,v)
    def on_left_arrow_release(self):
        print("Strafe Left")
        dir1 = (self.heading-90 +180)%(360) -180
        v = vel(dir1,self.speed)
        self.shv(self.heading,v)
    
        
if __name__ == "__main__":
    rospy.init_node("Controller")
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()