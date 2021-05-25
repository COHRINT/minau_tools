#!/usr/bin/env bash

if [ $# -eq 0 ]
  then
      echo "You must provide an IP"
else  
    export ROS_MASTER_URI=http://$1:11311
    local_info=(`hostname -I`)
    local_ip=${local_info[0]}
    export ROS_IP=$local_ip
    unset ROS_HOSTNAME
fi