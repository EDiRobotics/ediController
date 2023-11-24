#!/bin/bash

source /opt/ros/noetic/setup.bash
export ROS_HOSTNAME=192.168.1.240; export ROS_MASTER_URI=http://192.168.1.240:11311
roscore &

sleep 5

camera_numbers=$(ls /dev/video* | sed 's/[^0-9]*//g')

for camera_number in $camera_numbers
do
    /usr/bin/python3 get_images.py _camera_number:=$camera_number &
done

/usr/bin/python3 get_status.py &

wait
