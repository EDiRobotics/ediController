#!/usr/bin/env python
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


topic = '/sim_env/step/action'
action_publisher = rospy.Publisher(topic, String, queue_size=1)

service = '/env/step/policy_action'
rospy.wait_for_service(service)
action_service = rospy.ServiceProxy(service, StringService)

service = '/env/reset'
rospy.wait_for_service(service)
reset_service = rospy.ServiceProxy(service, Trigger)


def register_subscribers(image_topics, status_topic="/arm_status/all",
                         queue_size=1, status_cache_size=5, image_cache_size=10):
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
    rospy.loginfo(f"Obtain Camera topics: {str(camera_topics)}")
    return camera_topics


def obtain_obs_latest() -> (Dict, Dict):
    while not rospy.is_shutdown():
        timestamp_start, timestamp = rospy.Time.now() - Duration(0.1), rospy.Time.now()
        status = None
        for k, v in global_status_caches.items():
            cache = v
            cached_messages = cache.getInterval(timestamp_start, timestamp)
            status = fetch_status_from_msg(cached_messages)
            if status is None:
                continue

        images = {}
        for k, v in global_image_caches.items():
            cache = v
            cached_messages = cache.getInterval(timestamp_start, timestamp)
            img = fetch_img_from_msg(cached_messages, k)
            images[k] = img
            if img is None:
                continue
        else:
            return status, images

        # with ThreadPoolExecutor(max_workers=8) as executor:
        #     # Create a future for each item in global_image_caches
        #     futures = {k: executor.submit(process_single_cache, v, timestamp_start, timestamp, k) for k, v in
        #                global_image_caches.items()}
        #     images = {k: future.result() for k, future in futures.items()}


def obtain_obs_through_time(timestamp_start, timestamp) -> (Dict, Dict):
    status = None
    for k, v in global_status_caches.items():
        cache = v
        cached_messages = cache.getInterval(timestamp_start, timestamp)
        status = fetch_status_from_msg(cached_messages)
        if status is None:
            continue

    images = {}
    for k, v in global_image_caches.items():
        cache = v
        cached_messages = cache.getInterval(timestamp_start, timestamp)
        img = fetch_img_from_msg(cached_messages, k)
        images[k] = img

    return status, images

    # with ThreadPoolExecutor(max_workers=8) as executor:
    #     # Create a future for each item in global_image_caches
    #     futures = {k: executor.submit(process_single_cache, v, timestamp_start, timestamp, k) for k, v in
    #                global_image_caches.items()}
    #     images = {k: future.result() for k, future in futures.items()}


def process_single_cache(cache, timestamp, duration, name=None):
    cached_messages = cache.getInterval(timestamp - duration, timestamp)
    img = fetch_img_from_msg(cached_messages, name)
    return img


def fetch_img_from_msg(cached_messages, name=None):
    if not len(cached_messages) > 0:
        if name:
            rospy.logwarn(f"Img cached_messages of {name} is empty")
        else:
            rospy.logwarn(f"Img cached_messages is empty")
        return None
    img_msg = cached_messages[-1]
    img = cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    return img


def fetch_status_from_msg(cached_messages):
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


def publish_action(action):
    action_json = json.dumps(action)
    action_publisher.publish(String(action_json))
    rospy.logdebug(f"Published action: {action_json}")


def execute_action(action):
    action_json = json.dumps(action)
    request = StringServiceRequest(action_json)
    rospy.logdebug(f"Request action: {action_json}")
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
