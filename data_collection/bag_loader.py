import json
import os

import numpy as np
import rosbag
import rospy

rospy.init_node('rosbag_parser')


import json
import rosbag
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
import message_filters
import rospy

def load_from_bag(file_name):
    bag = rosbag.Bag(file_name)

    # Define topics of interest
    status_topic = "/arm_status/all"
    sensor_topics = [topic for topic, _ in bag.get_type_and_topic_info()[1].items() if topic.startswith("/sensor/")]
    action_topic = "/env/step/action"
    idx_topic = "/env/step/idx"

    # Initialize data structures
    topics = {status_topic: [], action_topic: [], idx_topic: []}
    for sensor_topic in sensor_topics:
        topics[sensor_topic] = []

    # Dictionary to store idx timestamps
    idx_times = {}

    # Process messages from the bag file
    for topic, msg, t in bag.read_messages(topics=topics.keys()):
        if topic == idx_topic:
            idx_times[msg.data] = t
            topics[topic].append((msg.data, t))
        elif topic == action_topic:
            action_data = json.loads(msg.data)
            topics[topic].append((action_data, t))
        else:
            # For other topics, use the timestamp of the message
            topics[topic].append((msg, t))

    # Process and pair actions with observations
    results = []
    for action_data, action_time in topics[action_topic]:
        idx = action_data["idx"]
        start_time = idx_times.get(idx - 1, None)
        end_time = action_time

        observations = {
            "status": [],
            "sensors": {sensor_topic: [] for sensor_topic in sensor_topics}
        }

        # Collect status and sensor data between idx timestamps
        for topic, messages in topics.items():
            if topic == status_topic or topic in sensor_topics:
                for msg, msg_time in messages:
                    if start_time is not None and msg_time > start_time and msg_time <= end_time:
                        if topic == status_topic:
                            observations["status"].append(msg)
                        else:
                            observations["sensors"][topic].append(msg)

        results.append({
            "idx": idx,
            "action": action_data,
            "observations": observations
        })

    bag.close()

    del results[0]
    for item in results:
        idx = item["idx"]
        observations = item["observations"]
        messages = observations["status"]
        if len(messages) == 0:
            print(f"idx {idx} status is empty")
        for k, messages in observations["sensors"].items():
            if len(messages) == 0:
                print(f"idx {idx} {k} is empty")

    return results



def find_files_with_extension(directory, extension):
    file_list = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                file_list.append(os.path.join(root, file))
    return file_list


file_dir = "./bag/"
extension = ".bag"
file_list = find_files_with_extension(file_dir, extension)

print(file_list)
for full_file_name in file_list:
    print(f"Loading {full_file_name}...")
    file_name = os.path.basename(full_file_name)
    name = os.path.splitext(file_name)[0]
    folder_name = os.path.dirname(full_file_name)
    results = load_from_bag(full_file_name)

print()
