import traceback
import sys
import time

sys.path.append("..")
try:
    from .edi_env_ros_interface import *

    rospy_enable = True
except:
    traceback.print_exc()
    time.sleep(1)
    rospy_enable = False
    print("Error on importing rospy...")
    print("You can still test other unrelated functions...")
from typing import Dict, List
import json
import numpy as np


# import gym


# class EdiEnv(gym.Env):
class EdiEnv:
    _action_chunk = False

    def __init__(self) -> None:
        super(EdiEnv, self).__init__()

        self.gripper_pos = 0

        self.last_status = None
        self.last_images = {}
        self.image_topics = self._get_camera_topics()

        start_listening(self.image_topics)
        self._wait_until_ready()

        rospy.loginfo('Initialized EdiEnv.')

    def reset(self):
        """
        Resets the environment and returns the initial observation.

        Returns:
          obs (dict): The initial observation after reset.
            - obs["status"] (dict): The initial status data.
              Contains sensor readings, system state, etc.
            - obs["images"] (dict): The initial image data.
              The keys are camera names (str).
              The values are OpenCV images (np.ndarray) from each camera.
        """
        obs = dict()
        execute_reset()
        time_now = rospy.Time.now()
        s, images = self._obtain_obs_through_time(time_now)
        obs["status"] = s
        obs["images"] = images
        return obs

    def step(self, action):
        """
        Performs one step in the environment by applying an action.

        Args:
          action: The action to apply.

        Returns:
          obs (dict): The observation after taking the action.
            - obs["status"] (dict): The status data after the step.
              Contains sensor readings, system state, etc.
            - obs["images"] (dict): The image data after the step.
              The keys are camera names (str).
              The values are OpenCV images (np.ndarray) from each camera.
          reward (float): The reward from taking the action.
          done (bool): True if the episode has ended, False otherwise.
          info (dict): Debugging info.
            - info["timestamp"] (float): Timestamp of the step.
            - info["error"] int: If error occurs.
            - info["error_details"] list: The first three items are obtained from GetRobotErrorCode().
        """
        if isinstance(action, np.ndarray):
            action = action.tolist()
        self.gripper_pos = action[-1]
        step_action_info = self.step_with_action(action)
        obs = dict()
        time_now = rospy.Time.now()
        s, images = self._obtain_obs_through_time(time_now)
        obs["status"] = s
        obs["images"] = images
        reward = 0.0
        done = False
        info = {"timestamp": time_now.to_time()}
        info.update(step_action_info)
        return obs, reward, done, info

    def close(self):
        pass

    def _obtain_obs_through_time(self, timestamp, time_interval=0.2) -> (Dict, Dict):
        duration = Duration(time_interval)
        status = None
        for k, v in global_status_caches.items():
            cache = v
            cached_messages = cache.getInterval(timestamp - duration, timestamp)
            status = self._fetch_status_from_msg(cached_messages)
            if status is None:
                status = self.last_status
            self.last_status = status
        images = {}
        for k, v in global_image_caches.items():
            cache = v
            cached_messages = cache.getInterval(timestamp - duration, timestamp)
            img = self._fetch_img_from_msg(cached_messages)
            if img is None and k in self.last_images:
                img = self.last_images[k]
            images[k] = img
            self.last_images[k] = img
        if isinstance(status, dict):
            status["gripper_pos"] = self.gripper_pos

        return status, images

    def _wait_until_ready(self):
        while not rospy.is_shutdown():
            time_now = rospy.Time.now()
            status, images = self._obtain_obs_through_time(time_now)

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

    @staticmethod
    def _get_camera_topics():
        all_topics = rospy.get_published_topics()
        camera_topics = [topic for topic, _ in all_topics if topic.startswith('/camera')]
        rospy.loginfo(f"Obtain Camera topics: {str(camera_topics)}")
        return camera_topics

    @staticmethod
    def _fetch_img_from_msg(cached_messages):
        if not len(cached_messages) > 0:
            rospy.logwarn(f"Img cached_messages is empty")
            return None
        img_msg = cached_messages[-1]
        img = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        return img

    @staticmethod
    def _fetch_status_from_msg(cached_messages):
        if not len(cached_messages) > 0:
            rospy.logwarn(f"Status cached_messages is empty")
            return None
        cached_datas = [msg.data for msg in cached_messages]
        status_data = cached_datas[-1]
        try:
            status = json.loads(status_data)
        except json.JSONDecodeError as e:
            rospy.logwarn(f"Error json loads: {e}")
            return None
        return status

    @classmethod
    def step_with_action(cls, action: List):
        """
        :param action: 7 length list. The first 6 item contain the 6 joint angle,
        the last item contains the gripper proportion.
        :return: a dict containing some information
        """
        # action = cls._action_chunk(action)
        action = [float(a) for a in action]
        joint = action[:6]
        gripper = action[-1]
        step_action_info = execute_action(action)
        return step_action_info

    @classmethod
    def _action_chunk(cls, action: List):
        """
        :param action: 7 length list. The first 6 item contain the 6 joint angle,
        the last item contains the gripper proportion.
        :return: a dict containing some information
        """
        joint = action[:6]
        gripper = action[-1:]
        return joint + gripper
