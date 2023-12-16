#!/bin/bash

#source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export ROS_HOSTNAME=192.168.1.240; export ROS_MASTER_URI=http://192.168.1.240:11311
echo "Set ROS_HOSTNAME=${ROS_HOSTNAME}, ROS_MASTER_URI=${ROS_MASTER_URI}..."

echo "Starting Demo Arm Publisher..."
python hardware/arm_demo.py &

wait
