import json
import os

import numpy as np
import rosbag
import rospy
import scipy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import re

rospy.init_node('rosbag_parser')


def load_from_bag(file_name):
    bag = rosbag.Bag(file_name)

    # Extract topic information, time transform
    topics = {}
    base_t = 0
    for i, (topic, msg, t) in enumerate(bag.read_messages()):
        t = t.to_sec()
        if i == 0:
            base_t = t
        if topic not in topics:
            topics[topic] = []
        topics[topic].append((msg, t - base_t))

    bag.close()

    # Fine Drone Numbers
    drone_topics = []
    for topic in topics:
        if re.search(r"/drone_\d+", topic):
            drone_topics.append(topic)
    drones = []
    for topic in drone_topics:
        drone_id = re.findall(r"\d+", topic)[-1]
        drones.append(int(drone_id))
    drone_nums = max(drones) + 1
    print("Total drones:", drone_nums)
    print()

    drone_nums = max(drones) + 1

    # Extract odom pose information
    odom_data = {}
    for i in range(drone_nums):
        odom_data[i] = []

    for topic, messages in topics.items():
        match = re.match(r"/drone_(\d+)_visual_slam/odom", topic)
        if not match:
            continue
        drone_id = int(match.group(1))
        for msg, t in messages:
            position = msg.pose.pose.position
            odom_data[drone_id].append(((position.x, position.y, position.z), t))

    max_times = []
    min_times = []
    for drone_id in odom_data:
        timestamps = [t for (_, t) in odom_data[drone_id]]
        max_time = 0
        min_time = float('inf')

        if timestamps:
            max_time = max(max_time, max(timestamps))
            min_time = min(min_time, min(timestamps))
        max_times.append(max_time)
        min_times.append(min_time)
    max_time = min(max_times)
    min_time = max(min_times)
    print("Max timestamp:", max_time)
    print("Min timestamp:", min_time)

    min_time_rounded = np.ceil(min_time)
    max_time_rounded = np.floor(max_time)
    time_step = 0.2
    times = np.arange(min_time_rounded, max_time_rounded, time_step)
    interp_data = {}
    for drone_id in odom_data:
        x_pos = []
        y_pos = []
        z_pos = []
        timestamps = []

        for p, t in odom_data[drone_id]:
            x_pos.append(p[0])
            y_pos.append(p[1])
            z_pos.append(p[2])
            timestamps.append(t)

        x_interp = scipy.interpolate.interp1d(timestamps, x_pos)
        y_interp = scipy.interpolate.interp1d(timestamps, y_pos)
        z_interp = scipy.interpolate.interp1d(timestamps, z_pos)

        interp_data[drone_id] = []
        for t in times:
            interp_data[drone_id].append((x_interp(t), y_interp(t), z_interp(t), t))
    # Interpolate as before

    interp_data_list = []
    for drone_id in interp_data:
        interp_data_list.append(interp_data[drone_id])

    interp_data_array = np.asarray(interp_data_list)
    return interp_data_array


def find_files_with_extension(directory, extension):
    file_list = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                file_list.append(os.path.join(root, file))
    return file_list


file_dir = "../bags/"
extension = ".bag"
file_list = find_files_with_extension(file_dir, extension)

for full_file_name in file_list:
    print(f"Loading {full_file_name}...")
    file_name = os.path.basename(full_file_name)
    name = os.path.splitext(file_name)[0]
    folder_name = os.path.dirname(full_file_name)
    positions = load_from_bag(full_file_name)

    np.save(os.path.join(folder_name, name), positions)
print()
# if "odom" in topic and isinstance(msg, String):
#     rospy.loginfo("Message: %s", msg.data)

"""
interp_data_array
    It is a NumPy 2D array with shape (num_drones, num_timepoints, 3)
    The first dimension indexes the drone ID (0 to num_drones-1)
    The second dimension is the interpolated time points (from min_time to max_time in 0.2 increments)
    The third dimension contains the x, y, z, t position and time at each time point (4 values)
"""
