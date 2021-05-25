#!/usr/bin/env python
import numpy as np
import rospy
from minau.srv import ArmControl, SetHeadingVelocity, SetHeadingDepth
from geometry_msgs.msg import Vector3
from minau.msg import ControlStatus
from sensor_msgs.msg import Joy

# def vel(heading,speed):
#     heading_rad = heading*np.pi/180
#     return Vector3(speed*np.cos(heading_rad),speed*np.sin(heading_rad),0)

class MyController():

    def __init__(self):
        # print(rospy.get_namespace())
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

        rospy.Subscriber("/joy", Joy, self.callback)
        # print(4)
        print("ready...")

        self.depth_flag = False
        self.heading_flag = False

    def callback(self, msg):
        
        cmd = False
        dx, dy = 0, 0
        
        if msg.buttons[2] and not self.depth_flag: # UP
            self.depth_flag = True
            self.depth -= 0.3
            self.shd(self.heading, self.depth)
            cmd = True
        if msg.buttons[0] and not self.depth_flag: # DOWN
            self.depth_flag = True
            self.depth += 0.3
            self.shd(self.heading, self.depth)
            cmd = True
        else:
            self.depth_flag = False

        
        heading_cmd = False
        if msg.buttons[3] and not self.heading_flag:
            new_heading = self.heading - 15
            while new_heading < 0:
                new_heading += 360
            while new_heading > 360:
                new_heading -= 360
            self.heading = new_heading
            self.heading_flag = True
            cmd = True
            heading_cmd = True
            
        elif msg.buttons[1] and not self.heading_flag:
            new_heading = self.heading + 15

            while new_heading < 0:
                new_heading += 360
            while new_heading > 360:
                new_heading -= 360
            self.heading = new_heading
            self.heading_flag = True
            cmd = True
            heading_cmd = True
        else:
            self.heading_flag = False

        # Y axis in real life (x in NED)
        if msg.axes[4] != 0:
            dy = self.speed * msg.axes[4]
        if msg.axes[3] != 0:
            dx = -self.speed * msg.axes[3] 
        if dx or dy or heading_cmd:
            vec = Vector3(dy, dx, 0.0)
            self.shv(self.heading, vec)
            cmd = True

        # print(msg)
            
        if cmd:
            print("Heading: {} | Depth: {} | dx: {} | dy: {}".format(self.heading, -self.depth, dx, dy))
    
        
if __name__ == "__main__":
    rospy.init_node("joy_controller")
    controller = MyController()
    rospy.spin()