#!/usr/bin/env python
"""
Contains class that does repeat testing with the configurations included in the input file. 
"""
import rospy
import sys
import csv
import pprint
import random, shutil
import os, signal
import subprocess
import numpy as np
import time
import math
import argparse
import yaml
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Bool


class repeatTester:
    """Automatically repeats the tests according to the configurations in the csv file"""
    def __init__(self):
        """Reads in the csv file and stores the information in variables belonging to the class to set up for the testing.
        Arguments:
            cFile {string} -- The csv file name that describes the configuations for the test.
        """
        self.data_loc = rospy.get_param("~data_loc")
        self.cFile = rospy.get_param("~file")
        #reads the csv in as a dictionary, removing all whitespace
        reader = csv.DictReader(
            open(self.cFile)
        )
        # remove leading and trailing whitespace from all values
        reader = (
            dict((k.strip(), v.strip()) for k, v in row.items() if v) for row in reader)
        self.config = []
        for row in reader:
            self.config.append(row)

        self.first_guppy = [5,0,-1]
        self.first_dory = [-5,0,-1]
        self.red_vel = 0.5
        self.blue_vel = 0.3
        self.debug = rospy.get_param("~debug",False)

        #extracts data and asigns it to variables
        self.config_names=[]
        self.custody = []
        for i in range(len(self.config)):
            self.config_names.append(self.config[i]['Test_Name'])
        self.test_group_name = self.config[0]['Test_Group_Name']
        self.num_groups = int(self.config[0]['Number_Tests_dep'])
        self.mins_per_test = int(self.config[0]['Mins_Per_Test_dep'])
        self.num_configs = len(self.config)
        self.total_tests = self.num_groups*self.num_configs
        self.remaining_time = self.mins_per_test*self.total_tests
        self.dim_x = float(self.config[0]['Map_Dim_x_dep'])
        self.dim_y = float(self.config[0]['Map_Dim_y_dep'])

        self.first_red = [2*self.dim_x,2*self.dim_y,-1]
        self.printIntro()
        self.Done = False


    def finished_callback(self,msg):
        self.Done = True
    def time(self):
        """Takes the time and converts it to a format that can be printed.
        Returns:
            [String] -- a string of the time of format XXh XXmin
        """
        t_formated = str(int(self.remaining_time/60))+'h '+str(self.remaining_time%60)+'min'
        return t_formated
    def printIntro(self):
        """Simply prints the intro"""        
        #prints initial information
        print('\n\nTest groups: '+str(self.num_groups))
        print('Total tests to execute: '+str(self.total_tests))
        print('Expected Duration: '+self.time()+'\n\n')
    
    def teleport(self):
        """Teleports assets to starting positions
        """
        set_model_state('guppy',self.first_guppy)
        set_model_state('dory',self.first_dory)
        set_model_state('red_actor_5',self.first_red)

    def run(self):
        """
        This is the function that runs all the tests and bags all the data.
        """
        #this is where the bagged data will be stored
        dirTo = self.data_loc+'/'+self.test_group_name
        os.mkdir(dirTo)
        shutil.copy(self.cFile,dirTo)
        for j in range(self.num_configs):
            for i in range(self.num_groups):
                print('\n\nExecuting '+self.test_group_name+'/'+self.config_names[j]+' #'+str(i+1)+' ('+str((i+1)+self.num_groups*j)+'/'+str(self.total_tests)+')\n\n')
                print('Time Remaining: '+ self.time())
                self.remaining_time -= self.mins_per_test
                self.teleport()

                FNULL = open(os.devnull, 'w')
                fileFor3 = self.data_loc+'/waypoints/'+'waypoints_'+str(i)+'.csv'
                args3 = ['rosrun','minau_tools','waypoint_move.py','__ns:=red_actor_5','_vel:='+str(self.red_vel),'_red:=true','_dimx:='+str(self.dim_x),'_dimy:='+str(self.dim_y),'_z:=-3']

                pose_guppy = 'pose_guppy:='+str(self.first_guppy)
                pose_dory = 'pose_dory:='+str(self.first_dory)
                args2 = ['roslaunch','minau_tools','uuv_etddf.launch',pose_guppy,pose_dory]


                args4 = ['rosrun','minau_tools','waypoint_move.py','__ns:=guppy','_vel:='+str(self.blue_vel),'_red:=false','_dimx:='+str(self.dim_x),'_dimy:='+str(self.dim_y),'_z:=-1']
                args5 = ['rosrun','minau_tools','waypoint_move.py','__ns:=dory','_vel:='+str(self.blue_vel),'_red:=false','_dimx:='+str(self.dim_x),'_dimy:='+str(self.dim_y),'_z:=-2']
                


                bagfile_name = self.config_names[j]+'_'+str(i+1)
                args6 = 'rosbag record -O '+bagfile_name+' /guppy/pose_gt \
                                                           /dory/pose_gt \
                                                           /red_actor_5/pose_gt \
                                                           /guppy/etddf/estimate/network \
                                                           /dory/etddf/estimate/network'
                if self.debug:
                    proc2 = subprocess.Popen(args2)
                else:
                    proc2 = subprocess.Popen(args2,stdout=FNULL,stderr=subprocess.STDOUT)
                time.sleep(10)
                proc6 = subprocess.Popen(args6,stdin=subprocess.PIPE, shell=True, cwd=dirTo)
                if self.debug:
                    proc3 = subprocess.Popen(args3)
                    proc4 = subprocess.Popen(args4)
                    proc5 = subprocess.Popen(args5)
                else:
                    proc5 = subprocess.Popen(args5,stdout=FNULL,stderr=subprocess.STDOUT)
                    proc3 = subprocess.Popen(args3,stdout=FNULL,stderr=subprocess.STDOUT)
                    proc4 = subprocess.Popen(args4,stdout=FNULL,stderr=subprocess.STDOUT)
                

                rospy.sleep(self.mins_per_test*60)


                terminate_ros_node("/record")
                proc3.terminate()
                proc4.terminate()
                proc2.terminate()
                proc5.terminate()
                time.sleep(10)

        print('All tests complete')
        print('Data located in data/'+self.test_group_name)

def set_model_state(model_name, pos):
    """Moves a model in gazebo
    Arguments:
        model_name {String} -- The name of what you want to move
        pos {[float,float,float]} -- Position of where you want to move it to
    """
    pose = Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    rospy.wait_for_service('/gazebo/set_model_state')    
    for i in range(3): # repeat 3 times, sometimes gazebo doesn't actually move the model but indicates it does in its modelstate...    
        result = None
        try:
            mover = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            res = mover(ModelState(model_name, pose, Twist(), "world") )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        time.sleep(0.1)
                

def terminate_ros_node(s):
    """This is used to stop the bagging correctly.
    Arguments:
        s {string} -- the prefix of the process you want to stop
    """
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode
    for str in list_output.split("\n"):
        if (str.startswith(s)):
            os.system("rosnode kill " + str)



if __name__ == "__main__":
    rospy.init_node('repeat_tester')
    rt = repeatTester()
    rt.run()