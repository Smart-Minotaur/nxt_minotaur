#!/bin/bash

BEAGLE_IP=192.168.1.1

export ROS_IP=192.168.1.100
export ROS_MASTER_URI=http://$BEAGLE_IP:11311
roslaunch nxt_beagle move_base.launch