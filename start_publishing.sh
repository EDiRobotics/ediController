#!/bin/bash

pkill -9 ros
echo "Killing previous ros programs..."
sleep 2
#source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export ROS_HOSTNAME=192.168.1.240; export ROS_MASTER_URI=http://192.168.1.240:11311
echo "Set ROS_HOSTNAME=${ROS_HOSTNAME}, ROS_MASTER_URI=${ROS_MASTER_URI}..."

echo "Starting roscore..."
roscore &

sleep 1
camera_numbers=$(ls /dev/video* | sed 's/[^0-9]*//g')
echo "Starting to obtain camera images, cameras $(echo $camera_numbers | tr '\n' ' ') are detected..."
for camera_number in $camera_numbers
do
    /usr/bin/python3 hardware/get_images.py _camera_number:=$camera_number &
done

echo "Starting to obtain arm status..."
/usr/bin/python3 hardware/get_status.py &

#echo "Starting recording progress..."
#/usr/bin/python3 edi_data_collection/recorder.py &

#echo "Starting Sim Environment..."
#python hardware/simulator.py &

#echo "Starting Real Environment Backend..."
#python hardware/fr5_backend.py &

wait
