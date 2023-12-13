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

global_image_caches = {}
global_status_caches = {}
cv_bridge = CvBridge()


def _listener():
    rospy.spin()


rospy.init_node('ros_interface', anonymous=True)
topic = '/sim_env/action'
action_publisher = rospy.Publisher(topic, String, queue_size=1)


def start_listening(image_topics, status_topic="/arm_status"):
    cache_size = 10
    sub = message_filters.Subscriber(status_topic, String)
    cache = message_filters.Cache(sub, cache_size, allow_headerless=True)
    global_status_caches[status_topic] = cache

    for image_topic in image_topics:
        cache_size = 10
        sub = message_filters.Subscriber(image_topic, Image)
        cache = message_filters.Cache(sub, cache_size)
        global_image_caches[image_topic] = cache

    thread = threading.Thread(target=_listener)
    thread.start()


def publish_action(action):
    action_json = json.dumps(action)
    action_publisher.publish(String(action_json))
    rospy.logdebug(f"Published action: {action_json}")
