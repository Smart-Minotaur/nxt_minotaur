#!/bin/bash

BEAGLE_IP=192.168.1.1
PC_IP=192.168.1.100

export ROS_IP=$PC_IP
export ROS_MASTER_URI=http://$PC_IP:11311
roslaunch nxt_beagle move_base.launch