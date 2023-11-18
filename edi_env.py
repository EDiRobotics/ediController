# import gym
import time
import rospy
from rospy import Duration
from typing import Dict
import json
from edi_env_ros_interface import *


# class EdiEnv(gym.Env):
class EdiEnv():
    def __init__(self) -> None:
        super(EdiEnv, self).__init__()
        self.last_status = None
        self.last_images = {}
        self.image_topics = self._get_camera_topics()

        start_listening(self.image_topics)
        self._wait_until_ready()
        rospy.loginfo('Initialized EdiEnv.')

    def reset(self):
        """
        Resets the environment to its initial state and returns the initial observation.
        """
        obs = dict()
        time_now = rospy.Time.now()
        s, images = self._obtain_obs_through_time(time_now)
        obs["status"] = s
        obs["images"] = images
        return obs

    def step(self, action):
        """
        Executes a step in the environment by applying an action.
        :param action: action
        :return: new observation, reward, completion status, and other info.
        """
        step_action_info = self._step_with_action(action)
        obs = dict()
        time_now = rospy.Time.now()
        s, images = self._obtain_obs_through_time(time_now)
        obs["status"] = s
        obs["images"] = images
        reward = 0.0
        done = False
        info = {"timestamp": time_now.to_time()}
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

    def _step_with_action(self, action):
        # TODO: Realworld Arm Control
        step_action_info = {}
        return step_action_info


env = EdiEnv()
while not rospy.is_shutdown():
    print(env.reset()["images"])
    time.sleep(3)
