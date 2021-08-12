#!/usr/bin/env python
from __future__ import division
import sys
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int16
from geometry_msgs.msg import Pose
from etddf_minau.msg import Measurement, MeasurementPackage
from etddf_minau.srv import GetMeasurementPackage
import minau.srv
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

NUM_BYTES = 32
COMPRESSION = False

DEFAULT_GLOBAL_POSE = [0.0,0.0,0.0,0.0]
DEFAULT_BEACON_NAME = 'wamv_1'
DEFAULT_BLUE_ASSET_NAMES = ["bruce", "guppy", "dory", "squirt", "bubbles"]
DEFAULT_RED_ASSET_NAMES = ["red1"]

class SeatracSnub:

    def __init__(self, beacon_name, robot_names, global_pose):
        self.beacon_name = beacon_name
        self.poses = {}
        beacon_pose = Pose()
        beacon_pose.position.x = global_pose[0]
        beacon_pose.position.y = global_pose[1]
        beacon_pose.position.z = global_pose[2]
        self.poses[self.beacon_name] = beacon_pose
        self.poses[self.beacon_name].orientation.w = 1
        for r in robot_names:
            rospy.Subscriber(r + "/pose_gt", Odometry, callback = self.pose_callback, callback_args = r)
            rospy.wait_for_message(r + "/pose_gt", Odometry)

    def pose_callback(self, msg, robot_name):
        self.poses[robot_name] = msg.pose.pose

def main(args):
    rospy.init_node("seatrac_snub", anonymous=True)

    parser = argparse.ArgumentParser(description='Setpoint visitor')
    parser.add_argument("-c", "--comms", type=str, help="if set then allow comms", required=False)
    parser.add_argument("-t", "--beaconName", dest="beacon_name", type=str, default=DEFAULT_BEACON_NAME, help="beacon name string")
    parser.add_argument("-b", "--blueTeamNames", dest="blue_team", nargs='+', default=DEFAULT_BLUE_ASSET_NAMES, help="blue asset names")
    parser.add_argument("-r", "--redTeamNames", dest="red_team", nargs='+', default=DEFAULT_RED_ASSET_NAMES, help="red asset names")
    args = parser.parse_args()
    comms = args.comms

    # Get the name for the beacon from arguments
    beacon_name = args.beacon_name
    print('Beacon name: {}'.format(beacon_name))

    # Names of all of the blue assets
    blue_assets = args.blue_team
    print('Blue team names: {}'.format(blue_assets))

    # Names of all of the red assets
    red_assets = args.red_team
    print('Red team names: {}'.format(red_assets))

    # REMOVED: This screws with things for some reason
    """
    # Get global pose from surface communication
    get_global_pose = rospy.ServiceProxy(
        '/{}/surface_communication/get_global_pose'.format(beacon_name), minau.srv.GetGlobalPose)
    global_pose = get_global_pose()
    global_pose = [global_pose.x, global_pose.y, global_pose.z, global_pose.yaw]
    """
    global_pose = DEFAULT_GLOBAL_POSE
    print('Retrieved beacon pose of {}'.format(global_pose))

    # [comms_type, time_taken]
    # PING_DELAY = 2
    # BROADCAST_DELAY = 4
    # comm_scheme = [["ping_wamv_1_to_bruce", PING_DELAY], ["ping_wamv_1_to_guppy", PING_DELAY], ["broadcast_wamv_1",BROADCAST_DELAY], ["broadcast_bruce",BROADCAST_DELAY], ["broadcast_guppy",BROADCAST_DELAY]]
    # COMPRESSION = True

    PING_DELAY = 2.0
    BROADCAST_DELAY = 3.0
    # PING_DELAY = 1.0
    # BROADCAST_DELAY = 1.0
    first_group = ["dory", "guppy", "bruce"]
    second_group = ["squirt", "bubbles"]
    comm_scheme = [['ping_{}_to_{}'.format(beacon_name, name), PING_DELAY] for name in first_group] + [["broadcast_wamv_1",BROADCAST_DELAY]]
    # comm_scheme += [['broadcast_{}'.format(name), PING_DELAY] for name in first_group]
    comm_scheme += [['ping_{}_to_{}'.format(beacon_name, name), PING_DELAY] for name in second_group] + [["broadcast_wamv_1",BROADCAST_DELAY]]
    # comm_scheme += [['broadcast_{}'.format(name), PING_DELAY] for name in second_group]
    # TODO add sharing of assets

    # if comms == None:
    #     comm_scheme = [['ping_{}_to_{}'.format(beacon_name, name), PING_DELAY] for name in blue_assets] + [["broadcast_wamv_1",BROADCAST_DELAY]] # Simple
    #     # comm_scheme = [["ping_wamv_1_to_bruce", PING_DELAY], ["ping_wamv_1_to_guppy", PING_DELAY], ["broadcast_wamv_1",BROADCAST_DELAY], ["broadcast_bruce",BROADCAST_DELAY], ["broadcast_guppy",BROADCAST_DELAY]]

    #     # Weird schedule
    #     # comm_scheme = [["ping_wamv_1_to_guppy", PING_DELAY], ["broadcast_wamv_1",BROADCAST_DELAY], ["broadcast_guppy",BROADCAST_DELAY]]
    # else:
    #     comm_scheme = [['broadcast_{}'.format(name), BROADCAST_DELAY] for name in blue_assets]
    print(comm_scheme)

    # This may not be the right way to do this, but at least all assets have a unique ID
    asset_landmark_dict = {beacon_name : 0}
    for (i, name) in enumerate(blue_assets + red_assets):
        asset_landmark_dict[name] = i + 1
    print(asset_landmark_dict)

    event_pubs = {}

    meas_pkg_pub_dict = {}
    for a in blue_assets:
        meas_pkg_pub_dict[a] = rospy.Publisher(a + "/etddf/packages_in", MeasurementPackage, queue_size=10)
        event_pubs[a] = rospy.Publisher("/event_pubs/" + a, Int16, queue_size=10)

    event_pubs[beacon_name] = rospy.Publisher("/event_pubs/{}".format(beacon_name), Int16, queue_size=10)
    seasnub = SeatracSnub(beacon_name, blue_assets, global_pose)

    curr_index = 0

    wamv_1_meas_pkg = MeasurementPackage()
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
            dist = max( np.linalg.norm([diff_x, diff_y, diff_z]) + np.random.normal(0, RANGE_SD), 0)
            range_meas = Measurement("modem_range", t, action_executed_by, measured_asset, dist, RANGE_SD**2, global_pose, -1.0)

            if beacon_name in curr_action:
                latest_meas_pkg.src_asset = action_executed_by
                wamv_1_meas_pkg.measurements.append(range_meas)
                # include azimuth
                # diff_x, diff_y = diff_y, diff_x # transform to NED in gazebo
                # az_sd = ( 15*np.random.uniform() + 30 ) * (np.pi/180)
                ang = np.arctan2(diff_y, diff_x) #+ np.random.normal(0, az_sd)
                ang_deg = np.rad2deg(ang) + np.random.normal(0, AZIMUTH_SD)
                az_meas = Measurement("modem_azimuth", t, action_executed_by, measured_asset, ang_deg, AZIMUTH_SD**2, global_pose, -1.0)
                wamv_1_meas_pkg.measurements.append(az_meas)
            else:
                latest_meas_pkg.src_asset = action_executed_by
                latest_meas_pkg.measrements.append(range_meas)

        elif "broadcast" in curr_action:
            if beacon_name in curr_action:
                wamv_1_meas_pkg.src_asset = beacon_name
                rospy.loginfo("{} broadcasting".format(beacon_name))

                if COMPRESSION:
                    bytes_ = measPkg2Bytes(wamv_1_meas_pkg, asset_landmark_dict, NUM_BYTES)
                    wamv_1_meas_pkg = bytes2MeasPkg(bytes_, 0.0, asset_landmark_dict, global_pose)

                event_pubs[beacon_name].publish(Int16())
                for asset_key in meas_pkg_pub_dict.keys():
                    meas_pkg_pub_dict[asset_key].publish(wamv_1_meas_pkg)
                wamv_1_meas_pkg = MeasurementPackage()
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
                        meas_pkg = bytes2MeasPkg(bytes_, 0.0, asset_landmark_dict, global_pose)

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
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))