#!/bin/bash

source /opt/ros/noetic/setup.bash

roscore &

# 延迟以确保 roscore 有足够时间启动
sleep 5

camera_numbers=$(ls /dev/video* | sed 's/[^0-9]*//g')

for camera_number in $camera_numbers
do
    /usr/bin/python3 get_images.py _camera_number:=$camera_number &
done

/usr/bin/python3 get_status.py &

wait
