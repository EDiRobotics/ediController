import sys
import traceback
import time
import json
from typing import Dict, List
import numpy as np

sys.path.append(".")

import rospy

rospy.init_node('robot_data_collector')

from edi_gym.edi_env import *

import message_filters
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image

# Dictionary to map idx values to timestamps
idx_timestamps = {}
idx_lock = threading.Lock()
last_action_idx = 0
last_idx = 0


def save_data_for_imitation_learning(observations, action_data):
    print("------------------------")
    print(f"last_action_idx {last_action_idx}, last_idx {last_idx} ")
    print("------------------------")


class Recorder:

    def __init__(self) -> None:
        super(Recorder, self).__init__()
        self.image_topics = get_camera_topics()

        self.last_status = None
        self.last_images = {}

        register_subscribers(self.image_topics, queue_size=100, status_cache_size=100, image_cache_size=1000)
        rospy.Subscriber('/env/step/action', String, self.action_callback)
        rospy.Subscriber('/env/step/idx', Int32, self.idx_callback)

    def collect_observations(self, start_time, end_time):
        """
        Collects observations between the given start and end timestamps.
        """
        status, images = obtain_obs_through_time(start_time, end_time)
        if status is None:
            rospy.logerr(f"Status not found.")
            status = self.last_status
        self.last_status = status
        for k, img in images.items():
            if img is None and k in self.last_images:
                rospy.logerr(f"Image {k} not found.")
                img = self.last_images[k]
            images[k] = img
            self.last_images[k] = img
        return {"status": status, "sensors": images}

    def idx_callback(self, idx_msg):
        """
        Callback for when an idx is received.
        """
        global last_action_idx, last_idx
        with idx_lock:
            idx_timestamps[idx_msg.data] = rospy.Time.now()
            last_idx = idx_msg.data

    def action_callback(self, action_msg):
        """
        Callback for when an action is received.
        """
        global last_action_idx, last_idx

        # Extract action data from the JSON string
        action_data = json.loads(action_msg.data)

        # Extract the current idx from the action data
        current_idx = action_data.get('idx', None)
        last_action_idx = current_idx

        # Determine the time interval for data collection
        with idx_lock:
            start_time = idx_timestamps.get(current_idx - 1,
                                            rospy.Time.now() - Duration(0.1))  # Timestamp of the previous idx
            end_time = idx_timestamps.get(current_idx, None)  # Timestamp of the current idx

        # Ensure both timestamps are available
        if start_time and end_time:
            # Collect latest observations based on the index
            observations = self.collect_observations(start_time, end_time)

            # Combine data and save for imitation learning
            save_data_for_imitation_learning(observations, action_data)
        else:
            rospy.logerr(f"Timestamps for idx {current_idx} not found.")


recorder = Recorder()
rospy.spin()
