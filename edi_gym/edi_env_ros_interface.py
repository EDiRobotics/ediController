#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import json
import rospy
import message_filters
from std_msgs.msg import String
from rospy import Duration
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from backend.srv import StringService, StringServiceRequest, StringServiceResponse

global_image_caches = {}
global_status_caches = {}
cv_bridge = CvBridge()


def _listener():
    rospy.spin()


rospy.init_node('ros_interface', anonymous=True)
topic = '/sim_env/action'
action_publisher = rospy.Publisher(topic, String, queue_size=1)

service = '/env/step/policy_action'
rospy.wait_for_service(service)
action_service = rospy.ServiceProxy(service, StringService)

service = '/env/reset'
rospy.wait_for_service(service)
reset_service = rospy.ServiceProxy(service, Trigger)


def start_listening(image_topics, status_topic="/arm_status"):
    cache_size = 10
    sub = message_filters.Subscriber(status_topic, String, queue_size=1)
    cache = message_filters.Cache(sub, cache_size, allow_headerless=True)
    global_status_caches[status_topic] = cache

    for image_topic in image_topics:
        cache_size = 10
        sub = message_filters.Subscriber(image_topic, Image, queue_size=1)
        cache = message_filters.Cache(sub, cache_size)
        global_image_caches[image_topic] = cache

    thread = threading.Thread(target=_listener)
    thread.start()


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


def execute_reset():
    request = TriggerRequest()
    rospy.logdebug(f"Request reset")
    response = reset_service(request)
    response: TriggerResponse
    if not response.success:
        rospy.logerr(f"Request reset return errors: {response.message}")
