#!/usr/bin/env python
from __future__ import division
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int16
from geometry_msgs.msg import Pose
from etddf_minau.msg import Measurement, MeasurementPackage
from etddf_minau.srv import GetMeasurementPackage
import rospy
import numpy as np
from copy import deepcopy
import argparse

from cuquantization.quantize import measPkg2Bytes, bytes2MeasPkg

"""
This node serves as a snub for the seatrac modem
Generates:
    - range & azimuth to assets
    - manages communication of packets to other assets
"""
COMMS_ACTION_INDEX = 0
COMMS_TIME_INDEX = 1

RANGE_SD = 0.1
AZIMUTH_SD = 5

GLOBAL_POSE = [0,0,0,0]

NUM_BYTES = 32
COMPRESSION = False

class SeatracSnub:

    def __init__(self, robot_names):
        self.poses = {}
        self.poses["topside"] = Pose() # Assume topside beacon located (0,0,0)
        self.poses["topside"].orientation.w = 1
        for r in robot_names:
            rospy.Subscriber(r + "/pose_gt", Odometry, callback = self.pose_callback, callback_args = r)
            rospy.wait_for_message(r + "/pose_gt", Odometry)

    def pose_callback(self, msg, robot_name):
        self.poses[robot_name] = msg.pose.pose

if __name__ == "__main__":
    rospy.init_node("seatrac_snub", anonymous=True)

    parser = argparse.ArgumentParser(description='Setpoint visitor')
    parser.add_argument("-c", "--comms", type=str, help="if set then allow comms", required=False)
    args = parser.parse_args()
    comms = args.comms

    # [comms_type, time_taken]
    # PING_DELAY = 2
    # BROADCAST_DELAY = 4
    # comm_scheme = [["ping_topside_to_bluerov2_3", PING_DELAY], ["ping_topside_to_bluerov2_4", PING_DELAY], ["broadcast_topside",BROADCAST_DELAY], ["broadcast_bluerov2_3",BROADCAST_DELAY], ["broadcast_bluerov2_4",BROADCAST_DELAY]]
    COMPRESSION = True

    PING_DELAY = 2.0
    BROADCAST_DELAY = 3
    #if comms == True:
    if True:
        comm_scheme = [["ping_topside_to_bluerov2_3", PING_DELAY], ["ping_topside_to_bluerov2_4", PING_DELAY], ["broadcast_topside",BROADCAST_DELAY]] # Simple
        # comm_scheme = [["ping_topside_to_bluerov2_3", PING_DELAY], ["ping_topside_to_bluerov2_4", PING_DELAY], ["broadcast_topside",BROADCAST_DELAY], ["broadcast_bluerov2_3",BROADCAST_DELAY], ["broadcast_bluerov2_4",BROADCAST_DELAY]]
        
        # Weird schedule
        # comm_scheme = [["ping_topside_to_bluerov2_4", PING_DELAY], ["broadcast_topside",BROADCAST_DELAY], ["broadcast_bluerov2_4",BROADCAST_DELAY]]
    else:
        comm_scheme = [["broadcast_bluerov2_3",BROADCAST_DELAY], ["broadcast_bluerov2_4",BROADCAST_DELAY]]
    print(comm_scheme)


    asset_landmark_dict = {"topside" : 0, "bluerov2_3":1, "bluerov2_4" : 2, "red_actor_5" : 3}

    event_pubs = {}

    assets = ["bluerov2_3", "bluerov2_4"]
    meas_pkg_pub_dict = {}
    for a in assets:
        meas_pkg_pub_dict[a] = rospy.Publisher(a + "/etddf/packages_in", MeasurementPackage, queue_size=10)
        event_pubs[a] = rospy.Publisher("/event_pubs/" + a, Int16, queue_size=10)

    event_pubs["topside"] = rospy.Publisher("/event_pubs/topside", Int16, queue_size=10)
    seasnub = SeatracSnub(assets)

    curr_index = 0

    topside_meas_pkg = MeasurementPackage()
    latest_meas_pkg = MeasurementPackage()

    while not rospy.is_shutdown():
        curr_comms = comm_scheme[curr_index]
        curr_action = curr_comms[COMMS_ACTION_INDEX]
        t = rospy.get_rostime()

        if "ping" in curr_action:
            new_str = curr_action[len("ping_"):]
            action_executed_by = new_str[ :new_str.index("_to_")]
            measured_asset = new_str[new_str.index("_to_") + len("_to_"):]

            print(action_executed_by + " pinging " + measured_asset)
            
            action_executed_by_pose = seasnub.poses[action_executed_by]
            measured_asset_pose = seasnub.poses[measured_asset]

        
            diff_x =measured_asset_pose.position.x - action_executed_by_pose.position.x
            diff_y =measured_asset_pose.position.y - action_executed_by_pose.position.y
            diff_z =measured_asset_pose.position.z - action_executed_by_pose.position.z
            dist = np.linalg.norm([diff_x, diff_y, diff_z]) + np.random.normal(0, RANGE_SD)
            range_meas = Measurement("modem_range", t, action_executed_by, measured_asset, dist, RANGE_SD**2, GLOBAL_POSE, -1.0)
            
            if "topside" in curr_action:
                latest_meas_pkg.src_asset = action_executed_by
                topside_meas_pkg.measurements.append(range_meas)
                # include azimuth
                # diff_x, diff_y = diff_y, diff_x # transform to NED in gazebo
                # az_sd = ( 15*np.random.uniform() + 30 ) * (np.pi/180)
                ang = np.arctan2(diff_y, diff_x) #+ np.random.normal(0, az_sd)
                ang_deg = np.rad2deg(ang) + np.random.normal(0, AZIMUTH_SD)
                az_meas = Measurement("modem_azimuth", t, action_executed_by, measured_asset, ang_deg, AZIMUTH_SD**2, GLOBAL_POSE, -1.0)
                topside_meas_pkg.measurements.append(az_meas)
            else:
                latest_meas_pkg.src_asset = action_executed_by
                latest_meas_pkg.measrements.append(range_meas)

        elif "broadcast" in curr_action:
            if "topside" in curr_action:
                topside_meas_pkg.src_asset = "topside"
                rospy.loginfo("topside broadcasting")

                if COMPRESSION:
                    bytes_ = measPkg2Bytes(topside_meas_pkg, asset_landmark_dict, NUM_BYTES)
                    topside_meas_pkg = bytes2MeasPkg(bytes_, 0.0, asset_landmark_dict, GLOBAL_POSE)

                event_pubs["topside"].publish(Int16())
                for asset_key in meas_pkg_pub_dict.keys():
                    meas_pkg_pub_dict[asset_key].publish(topside_meas_pkg)
                topside_meas_pkg = MeasurementPackage()
            else:
                
                agent = curr_action[len("broadcast_"):]
                rospy.loginfo("{} broadcasting".format(agent))
                rospy.wait_for_service(agent + "/etddf/get_measurement_package")
                gmp = rospy.ServiceProxy(agent + "/etddf/get_measurement_package", GetMeasurementPackage)
                rospy.loginfo("Acquired service")
                try:
                    meas_pkg = gmp().meas_pkg
                    orig_meas_pkg = deepcopy(meas_pkg)

                    if COMPRESSION:
                        rospy.loginfo("Compressing")
                        bytes_ = measPkg2Bytes(meas_pkg, asset_landmark_dict, NUM_BYTES)
                        meas_pkg = bytes2MeasPkg(bytes_, 0.0, asset_landmark_dict, GLOBAL_POSE)

                    print("#"*20)
                    num_bursts = 0
                    num_explicit = 0
                    explicit_meas = []
                    for msg in meas_pkg.measurements:
                        if "burst" in msg.meas_type:
                            num_bursts += 1
                        else:
                            num_explicit += 1
                            meas_type = "{}_{}_{}".format(msg.meas_type, msg.src_asset, msg.measured_asset)
                            explicit_meas.append( [meas_type, msg.data, msg.stamp.to_sec()] )
                    print("Delta Tier: {}".format(meas_pkg.delta_multiplier))
                    print("Num bursts: {} |    Num explicit: {}".format(num_bursts, num_explicit))
                    print(explicit_meas)

                    # DEBUG THE COMPRESSION
                    # for i in range(len(orig_meas_pkg.measurements)):
                    #     print(orig_meas_pkg.measurements[i])
                    #     print(meas_pkg.measurements[i])
                    #     print("---")
                    # meas_pkg = orig_meas_pkg
                    event_pubs[agent].publish(Int16(meas_pkg.delta_multiplier))
                    for asset_key in meas_pkg_pub_dict.keys():
                        if asset_key != agent:
                            print("publishing to: " + asset_key)
                            rospy.sleep(0.1)
                            meas_pkg_pub_dict[asset_key].publish(meas_pkg)
                            rospy.sleep(0.1)
                except rospy.ServiceException as e:
                    print(e)
                
                # break

        curr_index = (curr_index + 1) % len(comm_scheme)
        rospy.sleep( curr_comms[COMMS_TIME_INDEX] )
        