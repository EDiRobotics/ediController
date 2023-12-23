import sys
import traceback
import time
import json
from typing import Dict, List
import numpy as np
import rospy

sys.path.append(".")

try:
    from edi_gym.edi_env_ros_interface import *
except:
    traceback.print_exc()
    time.sleep(1)
    print("Error on importing rospy...")
    exit(1)


class EdiEnv:
    demo = False
    _action_chunk = False

    def __init__(self, demo=False) -> None:
        super(EdiEnv, self).__init__()
        EdiEnv.demo = demo

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
        obs = dict()
        execute_reset()
        if not self.demo:
            rospy.set_param("/env/ctrl/switch", "policy")
        time_now = rospy.Time.now()
        s, images = self._obtain_obs_latest()
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
        step_start = rospy.Time.now()
        step_action_info = self.step_with_action(action)
        obs = dict()
        obtain_start = rospy.Time.now()
        s, images = self._obtain_obs_latest()
        obtain_end = rospy.Time.now()
        obs["status"] = s
        obs["images"] = images
        reward = 0.0
        done = False
        info = {"timestamp_before_action": step_start.to_time(),
                "timestamp_after_action": obtain_start.to_time()}
        info.update(step_action_info)
        rospy.logdebug(f"[Gym: step] Step_start {step_start}, Obtain_start {obtain_start}, ")
        rospy.logdebug(f"[Gym: step] Action time {(obtain_start - step_start).to_sec()}")
        rospy.logdebug(f"[Gym: step] Obtain time {(obtain_end - obtain_start).to_sec()}")
        rospy.logdebug(f"[Gym: step] Package time {(rospy.Time.now() - obtain_end).to_sec()}")
        return obs, reward, done, info

    def close(self):
        pass

    def _obtain_obs_latest(self) -> (Dict, Dict):
        status, images = obtain_obs_latest()
        if status is None:
            rospy.logerr(f"[Serious] Status is None, which should not happen")
            status = self.last_status
        self.last_status = status
        for k, img in images.items():
            if img is None and k in self.last_images:
                rospy.logerr(f"[Serious] Image {k} is None, which should not happen")
                img = self.last_images[k]
            images[k] = img
            self.last_images[k] = img
        return status, images

    def _wait_until_ready(self):
        while not rospy.is_shutdown():
            rospy.loginfo('Checking Availability.')
            status, images = self._obtain_obs_latest()

            if status is None:
                rospy.logerr("Status is None, which should not happen")
                self.last_status = status

            for k, v in images.items():
                if v is None:
                    rospy.logerr(f"Image {k} is None, which should not happen")
                    self.last_images[k] = v

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
        # action = cls._action_chunk(action)
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
