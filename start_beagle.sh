#!/bin/bash

BEAGLE_IP=141.37.31.129

export ROS_IP=$BEAGLE_IP
export ROS_MASTER_URI=http://$BEAGLE_IP:11311
roslaunch nxt_beagle control.launch
