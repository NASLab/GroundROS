#!/bin/bash

echo "Setting remote access for agent $1"
export ROS_MASTER_URI=http://192.168.$1.11:11311
echo "ROS_MASTER_URI is set to" $ROS_MASTER_URI