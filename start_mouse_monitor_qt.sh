#!/bin/bash

BEAGLE_IP=141.37.31.128
PC_IP=141.37.31.101

export ROS_IP=$PC_IP
export ROS_MASTER_URI=http://$PC_IP:11311
roslaunch nxt_qt mousemonitor.launch
