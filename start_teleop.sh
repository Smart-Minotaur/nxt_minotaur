#!/bin/bash

BEAGLE_IP=192.168.1.1

export ROS_IP=$BEAGLE_IP
export ROS_MASTER_URI=http://$BEAGLE_IP:11311
roslaunch minotaur_pc teleop.launch