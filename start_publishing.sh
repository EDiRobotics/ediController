#!/bin/bash

pkill -9 ros
echo "Killing previous ros programs..."
sleep 2
source /opt/ros/noetic/setup.bash
export ROS_HOSTNAME=192.168.1.240; export ROS_MASTER_URI=http://192.168.1.240:11311
echo "Set ROS_HOSTNAME=${ROS_HOSTNAME}, ROS_MASTER_URI=${ROS_MASTER_URI}..."

echo "Starting roscore..."
roscore &

sleep 3
camera_numbers=$(ls /dev/video* | sed 's/[^0-9]*//g')
echo "Starting obtain camera images..."
for camera_number in $camera_numbers
do
    /usr/bin/python3 get_images.py _camera_number:=$camera_number &
done

echo "Starting obtain arm status..."
/usr/bin/python3 get_status.py &

wait
