#!/bin/bash

BEAGLE_IP=141.37.31.129
PC_IP=141.37.31.101

export ROS_IP=$BEAGLE_IP
export ROS_MASTER_URI=http://$PC_IP:11311
roslaunch nxt_beagle control.launch
