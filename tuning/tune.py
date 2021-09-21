#!/usr/bin/env python

"""
Subscribes to raw sensor data: estimate bias (offset) and variance
Sensors
- baro
- dvl
- compass
- gyro
- accel
- modem range
- modem depth
"""

import argparse
from unicodedata import name
import rospy
from rospy.client import init_node
import tf
import numpy as np
import yaml

from std_msgs.msg import Float64
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from etddf_minau.msg import MeasurementPackage
from cuprint.cuprint import CUPrint

class Subber:

    def __init__(self, name):
        rospy.Subscriber("{}/mavros/global_position/rel_alt".format(name), Float64, self.baro_callback)
        rospy.Subscriber("{}/dvl".format(name), TwistWithCovarianceStamped, self.dvl_callback)
        rospy.Subscriber("{}/imu/data".format(name), Imu, self.compass_callback)
        rospy.Subscriber("{}/etddf/packages_in".format(name), MeasurementPackage, self.modem_callback)
        self.cuprint = CUPrint("tuner")

        self.baro_msgs = []
        self.dvl_x_msgs = []
        self.dvl_y_msgs = []
        self.dvl_z_msgs = []

        # Compass
        self.compass_roll_msgs = []
        self.compass_pitch_msgs = []
        self.compass_yaw_msgs = []
        self.gyro_x_msgs = []
        self.gyro_y_msgs = []
        self.gyro_z_msgs = []
        self.accel_x_msgs = []
        self.accel_y_msgs = []
        self.accel_z_msgs = []

        # Modem
        self.range_msgs = []
        self.azimuth_msgs = []

    def baro_callback(self, msg):
        self.baro_msgs.append( msg.data )

    def dvl_callback(self, msg):
        self.dvl_x_msgs.append( msg.twist.twist.linear.x )
        self.dvl_y_msgs.append( msg.twist.twist.linear.y )
        self.dvl_z_msgs.append( msg.twist.twist.linear.z )

        msgs = " baro {}| dvl {}| compass {}| modem {}".format(len(self.baro_msgs), len(self.dvl_x_msgs), len(self.compass_roll_msgs), len(self.range_msgs))
        self.cuprint(msgs, print_prev_line=True)

    def compass_callback(self, msg):

        # Compass
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, \
                msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.compass_roll_msgs.append( r )
        self.compass_pitch_msgs.append( p )
        self.compass_yaw_msgs.append( y )

        # Gyro
        self.gyro_x_msgs.append( msg.angular_velocity.x )
        self.gyro_y_msgs.append( msg.angular_velocity.y )
        self.gyro_z_msgs.append( msg.angular_velocity.z )

        # Acceleration
        self.accel_x_msgs.append( msg.linear_acceleration.x )
        self.accel_y_msgs.append( msg.linear_acceleration.y )
        self.accel_z_msgs.append( msg.linear_acceleration.z )

    def modem_callback(self, msg):
        robot_name = rospy.get_namespace().strip("/")
        for m in msg.measurements:
            if robot_name in m.measured_asset:
                if m.meas_type == "modem_azimuth":
                    self.azimuth_msgs.append( m.data )
                elif m.meas_type == "modem_range":
                    self.range_msgs.append( m.data )                    

        

# Assume the modem is located at [0,0,depth]
# Assume the vehicle is located at [0, y, 0]

parser = argparse.ArgumentParser(description='Bias and Variance Calculator of sensors.\nModem Pose=[0,0,depth,0]\nVehicle Position=[0,y,0]')
parser.add_argument("-n", "--name", type=str, help="Name of asset",required=True)
parser.add_argument("-t", "--time", type=int, help="seconds to collect data for",default=60, required=False)
parser.add_argument("-m", "--modem", type=bool, help="whether or not to collect modem data",default=True, required=False)
parser.add_argument("-d", "--depth", type=float, help="Depth of the modem", default=0.0, required=False)
parser.add_argument("-y", "--y_pos", type=float, help="Y position of the vehicle", default=0.0, required=True)
args = parser.parse_args()

# Truth (expected) + bias = measured --> bias = measured - truth
# Truth = measured - bias

modem_pose = [0,0, args.depth, 0] # [m, m, m, rad]
vehicle_position = [0,args.y_pos, 0]
modem_position=np.array(modem_pose)[:-1] # remove the pose
vehicle_position=np.array(vehicle_position)
delta_position = vehicle_position - modem_position
expected_range = np.linalg.norm( delta_position )
expected_azimuth_rad = np.arctan2(delta_position[1], delta_position[0]) - modem_pose[-1]
expected_azimuth_deg = np.degrees(expected_azimuth_rad)
print("Expecting Range: {} | Az: {}".format(expected_range, expected_azimuth_deg))

# Modem should be measuring 90 deg

rospy.init_node("tuner")
print("Collecting data for {}s".format(args.time))
print("Jump on the surface modem a few times to simulate waves\n")
s = Subber(args.name)
rospy.sleep(args.time)

# Produce the yaml file
try:
    yaml_obj = {
        "surface_beacon_name" : "topside",
        "surface_beacon_position" : [0,0, args.depth],
        "baro" : 
            {
                "bias" : float( np.mean( np.array(s.baro_msgs) ) ),
                "var" : float( np.var( np.array(s.baro_msgs) ) )
            },
        "dvl_x" : 
            {
                "bias" : float( np.mean(np.array(s.dvl_x_msgs)) ),
                "var" : float( np.var(np.array(s.dvl_x_msgs)) )
            },
        "dvl_y" : 
            {
                "bias" : float( np.mean(np.array(s.dvl_y_msgs)) ),
                "var" : float( np.var(np.array(s.dvl_y_msgs)) )
            },
        "dvl_z" : 
            {
                "bias" : float( np.mean(np.array(s.dvl_z_msgs)) ),
                "var" : float( np.var(np.array(s.dvl_z_msgs)) )
            },
        "modem_az" : 
            {
                "bias" : float( np.mean( np.array(s.azimuth_msgs) ) - expected_azimuth_deg ),
                "var" : float( np.var( np.array(s.azimuth_msgs)) )
            },
        "modem_range" : 
            {
                "bias" : float( np.mean( np.array(s.range_msgs) ) - expected_range ),
                "var" : float( np.var( np.array(s.range_msgs)) )
            }
        }
except Exception as exc:
    print("Missed messages. Unable to write file")
    import sys
    sys.exit(-1)

with open('config.yaml', 'w') as file:
    documents = yaml.dump({"mission_config" : yaml_obj}, file)

print() # Empty line

## Baro
data = np.array(s.baro_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Baro\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## DVL X
data = np.array(s.dvl_x_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Dvl x\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## DVL Y
data = np.array(s.dvl_y_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Dvl y\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## DVL Z
data = np.array(s.dvl_z_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Dvl z\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))
# print("*"*10 + "\n")

## Compass
data = np.array(s.compass_roll_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Compass roll\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## DVL Y
data = np.array(s.compass_pitch_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Compass pitch\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## DVL Z
data = np.array(s.compass_yaw_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Compass yaw\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))
# print("*"*10 + "\n")

## Gryo X
data = np.array(s.gyro_x_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Gyro x\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## Gyro Y
data = np.array(s.gyro_y_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Gyro y\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## Gyro Z
data = np.array(s.gyro_z_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Gyro z\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))
# print("*"*10 + "\n") 

## Accel X
data = np.array(s.accel_x_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Accel x\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## Accel Y
data = np.array(s.accel_y_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Accel y\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))

## Accel Z
data = np.array(s.accel_z_msgs)
num = len(data)
mean = round(np.mean(data),3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Accel z\t\tbias: {}\tstd: {}\tvar:{}\t({})".format(mean, std, var, num))
# print("*"*10 + "\n") 

## Modem Range
data = np.array(s.range_msgs)
num = len(data)
mean = np.mean(data)
bias = round( mean - expected_range, 3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Modem range\tbias: {}\tstd: {}\tvar:{}\t({})".format(bias, std, var, num))

## Modem Azimuth
data = np.array(s.azimuth_msgs)
num = len(data)
mean = np.mean(data)
bias = round( mean - expected_azimuth_deg, 3)
std = round(np.std(data),3)
var = round(np.var(data),3)
print("Modem azimuth\tbias: {}\tstd: {}\tvar:{}\t({})".format(bias, std, var, num))