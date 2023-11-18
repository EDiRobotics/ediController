#!/bin/bash

source /opt/ros/noetic/setup.bash

roscore &

# 延迟以确保 roscore 有足够时间启动
sleep 5

/usr/bin/python3 get_images.py &
/usr/bin/python3 get_status.py &

wait
