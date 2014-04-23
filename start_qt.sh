#!/bin/bash

BEAGLE_IP=141.37.31.129

export ROS_IP=141.37.31.101
export ROS_MASTER_URI=http://$BEAGLE_IP:11311
rosrun nxt_qt QPIDControl
