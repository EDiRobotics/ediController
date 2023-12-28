#!/usr/bin/env python
import time

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import json
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List

import rospy
import message_filters
from std_msgs.msg import String
from rospy import Duration
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from backend.srv import StringService, StringServiceRequest, StringServiceResponse

try:
    rospy.init_node('ros_interface', anonymous=True)
except:
    pass

global_image_caches = {}
global_status_caches = {}
cv_bridge = CvBridge()


def _listener():
    rospy.spin()


action_topic = '/sim_env/step/action'
action_publisher = rospy.Publisher(action_topic, String, queue_size=1)

demo_service_srv = '/env/step/demo_action_srv'
action_service_demo = rospy.ServiceProxy(demo_service_srv, StringService)
policy_service_srv = '/env/step/policy_action_srv'
action_service_policy = rospy.ServiceProxy(policy_service_srv, StringService)

reset_service_srv = '/env/reset_srv'
reset_service = rospy.ServiceProxy(reset_service_srv, Trigger)


def register_servers():
    rospy.wait_for_service(demo_service_srv)
    rospy.wait_for_service(policy_service_srv)
    rospy.wait_for_service(reset_service_srv)


def register_subscribers(image_topics, status_topic="/arm_status/all",
                         queue_size=1, status_cache_size=50, image_cache_size=50):
    cache_size = status_cache_size
    sub = message_filters.Subscriber(status_topic, String, queue_size=queue_size)
    cache = message_filters.Cache(sub, cache_size, allow_headerless=True)
    global_status_caches[status_topic] = cache

    for image_topic in image_topics:
        cache_size = image_cache_size
        sub = message_filters.Subscriber(image_topic, Image, queue_size=queue_size)
        cache = message_filters.Cache(sub, cache_size)
        global_image_caches[image_topic] = cache


def start_listening():
    thread = threading.Thread(target=_listener)
    thread.start()


def get_camera_topics():
    all_topics = rospy.get_published_topics()
    camera_topics = [topic for topic, _ in all_topics if topic.startswith('/sensor/camera')]
    if len(camera_topics) >0:
        rospy.loginfo(f"Obtain {len(camera_topics)} Camera topics: {str(camera_topics)}")
    else:
        rospy.logwarn(f"No camera topic detected.")

    return camera_topics


def process_single_cache(cache, timestamp_start, timestamp, name=None):
    cached_messages = cache.getInterval(timestamp_start, timestamp)
    img = fetch_img_from_msgs(cached_messages, name)
    return img


def process_single_cache_after(cache, timestamp_start, name=None):
    img = None
    latest = cache.getLatestTime()
    if latest is not None and timestamp_start < latest:
        cached_messages = [cache.getElemAfterTime(timestamp_start)]
        img = fetch_img_from_msgs(cached_messages, name)
    return img


def process_single_cache_latest(cache, timestamp_start, name=None):
    img = None
    latest = cache.getLatestTime()
    if latest is not None and timestamp_start < latest:
        cached_messages = [cache.getLast()]
        img = fetch_img_from_msgs(cached_messages, name)
    return img


def obtain_obs_latest(timeout=1) -> (Dict, Dict):
    timestamp_start = rospy.Time.now()

    while not rospy.is_shutdown():
        if rospy.Time.now() - timestamp_start > rospy.Duration(timeout):
            return None, {k: None for k in global_image_caches}
        status = None
        for _, cache in global_status_caches.items():
            cache: message_filters.Cache
            latest = cache.getLatestTime()
            if latest is not None and timestamp_start < latest:
                cached_messages = [cache.getLast()]
                status = fetch_status_from_msgs(cached_messages)

        with ThreadPoolExecutor(max_workers=8) as executor:
            futures = {k: executor.submit(process_single_cache_latest, cache, timestamp_start, k) for k, cache in
                       global_image_caches.items()}
            images = {k: future.result() for k, future in futures.items()}

        if status is not None and all(v is not None for v in images.values()):
            return status, images


def obtain_obs_after_time(timestamp_start) -> (Dict, Dict):
    while not rospy.is_shutdown():
        status = None
        for k, cache in global_status_caches.items():
            cache: message_filters.Cache
            latest = cache.getLatestTime()
            if latest is not None and timestamp_start < latest:
                cached_messages = [cache.getElemAfterTime(timestamp_start)]
                status = fetch_status_from_msgs(cached_messages)

        with ThreadPoolExecutor(max_workers=8) as executor:
            futures = {k: executor.submit(process_single_cache_after, cache, timestamp_start, k) for k, cache in
                       global_image_caches.items()}
            images = {k: future.result() for k, future in futures.items()}

        if status is not None and all(v is not None for v in images.values()):
            return status, images


def obtain_obs_through_time(timestamp_start, timestamp) -> (Dict, Dict):
    status = None
    for k, cache in global_status_caches.items():
        cache: message_filters.Cache
        cached_messages = cache.getInterval(timestamp_start, timestamp)
        status = fetch_status_from_msgs(cached_messages)

    with ThreadPoolExecutor(max_workers=8) as executor:
        futures = {k: executor.submit(process_single_cache, cache, timestamp_start, timestamp, k) for k, cache in
                   global_image_caches.items()}
        images = {k: future.result() for k, future in futures.items()}

    return status, images


def fetch_img_from_msgs(cached_messages, name=None):
    if not len(cached_messages) > 0:
        # if name:
        #     rospy.logwarn(f"Img cached_messages of {name} is empty")
        # else:
        #     rospy.logwarn(f"Img cached_messages is empty")
        return None
    img_msg = cached_messages[-1]
    img = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    return img


def fetch_status_from_msgs(cached_messages):
    if not len(cached_messages) > 0:
        # rospy.logwarn(f"Status cached_messages is empty")
        return None
    cached_datas = [msg.data for msg in cached_messages]
    status_data = cached_datas[-1]
    try:
        status = json.loads(status_data)
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Error json loads: {e}")
        return None
    return status


def publish_action(action):
    action_json = json.dumps(action)
    action_publisher.publish(String(action_json))
    rospy.logdebug(f"Published action: {action_json}")


def execute_action(action, demo=False):
    action_json = json.dumps(action)
    request = StringServiceRequest(action_json)
    rospy.logdebug(f"Request action: {action_json}")
    action_service = action_service_demo if demo else action_service_policy
    response = action_service(request)
    if not response.success:
        rospy.logerr(f"Request action return errors: {response.message}")
        if "allowed" in response.message.lower():
            return {"error": -1, "error_details": "Not allowed"}
    try:
        info = json.loads(response.message)
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Error json loads: {e}")
        return {}
    return info


def execute_reset():
    request = TriggerRequest()
    rospy.logdebug(f"Request reset")
    response = reset_service(request)
    response: TriggerResponse
    if not response.success:
        rospy.logerr(f"Request reset return errors: {response.message}")
