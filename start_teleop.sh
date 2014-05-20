#!/bin/bash

BEAGLE_IP=141.37.31.129
PC_IP=141.37.31.113

export ROS_IP=$PC_IP
export ROS_MASTER_URI=http://$PC_IP:11311
roslaunch minotaur_pc teleop.launch
