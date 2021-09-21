#!/usr/bin/env python

import argparse

parser = argparse.ArgumentParser(description='PID Setpoint placer')
parser.add_argument("-n", "--name", type=str, help="Name of asset",required=True)
parser.add_argument("-t", "--time", type=int, help="seconds to collect data for",default=60, required=False)
parser.add_argument("-m", "--modem", type=bool, help="whether or not to collect modem data",default=True, required=False)
parser.add_argument("-d", "--depth", type=float, help="Depth of the modem", default=0.0, required=False)
parser.add_argument("-y", "--y_pos", type=float, help="Y position of the vehicle", default=0.0, required=True)
args = parser.parse_args()