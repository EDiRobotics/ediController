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
import queue

# Dictionary to map idx values to timestamps
idx_timestamps = {}
idx_lock = threading.Lock()
last_action_idx = 0
last_idx = -1

action_queue = queue.Queue()


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

        register_subscribers(self.image_topics, queue_size=1000, status_cache_size=10000, image_cache_size=10000)
        self._wait_until_ready()

        rospy.Subscriber('/env/step/action', String, self.action_callback, queue_size=10)
        rospy.Subscriber('/env/step/idx', Int32, self.idx_callback, queue_size=10)
        self.action_thread = threading.Thread(target=self.process_actions)
        self.action_thread.daemon = True
        self.action_thread.start()

    def process_actions(self):
        """
        Processes actions from the queue.
        """
        global last_action_idx, last_idx

        while not rospy.is_shutdown():
            action_data_json = action_queue.get()

            action_data = json.loads(action_data_json)
            # Extract the current idx from the action data
            current_idx = action_data.get('idx', None)

            last_action_idx = current_idx
            start_time = None
            end_time = None
            retry = 0
            while not rospy.is_shutdown():
                start_time = idx_timestamps.get(current_idx - 1, None)  # Timestamp of the previous idx
                end_time = idx_timestamps.get(current_idx, None)  # Timestamp of the current idx
                if start_time and end_time:
                    break
                else:
                    if retry > 5:
                        break
                    # rospy.logwarn(f"Waiting for timestamp for idx {current_idx}...")
                    retry += 1
                    time.sleep(0.01)
            if not (start_time and end_time):
                rospy.logwarn(f"Timestamp for idx {current_idx} does not exist!")
                continue
            observations = None
            retry = 0
            while not rospy.is_shutdown():
                observations = self.collect_observations(start_time, end_time)
                if observations is not None:
                    save_data_for_imitation_learning(observations, action_data)
                    break
                else:
                    if retry > 10:
                        rospy.logerr(f"Observation for idx {current_idx} does not exist!")
                        break
                    retry += 1
                    time.sleep(0.01)

            # action_queue.task_done()

    def collect_observations(self, start_time, end_time):
        """
        Collects observations between the given start and end timestamps.
        """
        status, images = obtain_obs_through_time(start_time, end_time)
        if status is None:
            # rospy.logerr(f"Status not found.")
            return None
            status = self.last_status
        self.last_status = status
        for k, img in images.items():
            if img is None and k in self.last_images:
                # rospy.logerr(f"Image {k} not found.")
                return None
                img = self.last_images[k]
            images[k] = img
            self.last_images[k] = img
        return {"status": status, "sensors": images}

    def idx_callback(self, idx_msg):
        """
        Callback for when an idx is received.
        """
        global last_action_idx, last_idx
        idx_timestamps[idx_msg.data] = rospy.Time.now()
        if last_idx + 1 != idx_msg.data and last_idx != -1:
            rospy.logerr(f"Idx {last_idx + 1} missed")
        last_idx = idx_msg.data

    def action_callback(self, action_msg):
        """
        Callback for when an action is received.
        """
        action_queue.put(action_msg.data)

    def _wait_until_ready(self):
        while not rospy.is_shutdown():
            time_now = rospy.Time.now()
            status, images = self._obtain_obs_latest()

            if status is not None:
                self.last_status = status

            for k, v in images.items():
                if v is not None:
                    self.last_images[k] = v

            rospy.loginfo('Checking Availability.')

            if status is not None and all(v is not None for v in images.values()):
                break
            rospy.loginfo('Something not available, Retrying...')
            rospy.sleep(1)

    def _obtain_obs_latest(self) -> (Dict, Dict):
        status, images = obtain_obs_latest()
        if status is None:
            status = self.last_status
        self.last_status = status
        for k, img in images.items():
            if img is None and k in self.last_images:
                img = self.last_images[k]
            images[k] = img
            self.last_images[k] = img
        return status, images


recorder = Recorder()
rospy.spin()
