import sys
import traceback
import time
import json
from typing import Dict, List
import numpy as np
import rospy

sys.path.append(".")

from edi_gym.edi_env_ros_interface import *


class HeartbeatClient:
    def __init__(self, client_identifier="default", heartbeat_interval=2):
        self.heartbeat_interval = heartbeat_interval
        self.client_identifier = client_identifier
        self.publisher = rospy.Publisher('/record/ctrl/client', String, queue_size=10)
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        # self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()

    def send_heartbeat(self):
        while not rospy.is_shutdown():
            self.publisher.publish(self.client_identifier)
            # rospy.loginfo(f"Heartbeat sent by client {self.client_identifier}")
            time.sleep(self.heartbeat_interval)


class EdiEnv:
    demo = False
    _use_action_chunk = False

    def __init__(self, node_name="ros_interface", demo=False) -> None:
        super(EdiEnv, self).__init__()
        EdiEnv.demo = demo
        try:
            rospy.init_node(node_name, anonymous=True)
        except:
            pass

        self.heartbeat_client = HeartbeatClient()

        self.last_status = None
        self.last_images = {}
        self.image_topics = get_camera_topics()

        register_subscribers(self.image_topics)
        start_listening()
        self._wait_until_ready()
        self.reset()
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
        execute_reset()
        if not self.demo:
            rospy.set_param("/env/ctrl/switch", "policy")
        obs, _ = self._obtain_obs_latest()
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
        step_start = rospy.Time.now()
        step_action_info = self.step_with_action(action)
        obtain_start = rospy.Time.now()
        obs, timestamp = self._obtain_obs_latest()
        reward = 0.0
        done = False
        info = {"obs_timestamp": timestamp.to_time()}
        info.update(step_action_info)
        rospy.logdebug(f"[Gym: step] Step_start {step_start}, Obtain_start {obtain_start}, ")
        rospy.logdebug(f"[Gym: step] Action time {(obtain_start - step_start).to_sec()}")
        return obs, reward, done, info

    def close(self):
        pass

    def _obtain_obs_latest(self) -> (Dict, Dict):
        status, images = obtain_obs_latest()
        if status is None:
            rospy.logerr(
                f"[Serious] Status is None, which should not happen, please check the publishing nodes...")
            status = self.last_status
        self.last_status = status
        for k, img in images.items():
            if img is None and k in self.last_images:
                rospy.logerr(
                    f"[Serious] Image {k} is None, which should not happen, please check the publishing nodes...")
                img = self.last_images[k]
            images[k] = img
            self.last_images[k] = img
        obs = dict()
        obs["status"] = status
        obs["images"] = images
        return obs, rospy.Time.now()

    def _wait_until_ready(self):
        while not rospy.is_shutdown():
            rospy.loginfo('Checking Availability.')
            obs, _ = self._obtain_obs_latest()
            status = obs["status"]
            images = obs["images"]
            if status is not None and all(v is not None for v in images.values()):
                break
            rospy.loginfo('Something not available, Retrying...')
            rospy.sleep(1)

    @classmethod
    def step_with_action(cls, action: List):
        """
        :param action: 7 length list. The first 6 item contain the 6 joint angle,
        the last item contains the gripper proportion.
        :return: a dict containing some information
        """
        if cls._use_action_chunk:
            action = cls._action_chunk(action)
        action = [float(a) for a in action]
        step_action_info = execute_action(action, demo=cls.demo)
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
