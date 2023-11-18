#!/usr/bin/env python
import json

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import threading
import rospy
import message_filters
from std_msgs.msg import String
from rospy.rostime import Time

global_image_caches = {}
global_status_caches = {}
cv_bridge = CvBridge()


def _listener():
    rospy.spin()


rospy.init_node('ros_interface', anonymous=True)


def start_listening(image_topics, status_topic="/arm_status"):
    cache_size = 10
    sub = message_filters.Subscriber(status_topic, String)
    cache = message_filters.Cache(sub, cache_size, allow_headerless=True)
    global_status_caches[status_topic] = cache

    # subs = []
    # status_sub = rospy.Subscriber(status_topic, String, _status_callback)
    # subs.append(status_sub)

    for image_topic in image_topics:
        # callback_with_topic = lambda msg, topic=image_topic: _image_callback(msg, topic)
        # sub = rospy.Subscriber(image_topic, Image, callback_with_topic)
        # subs.append(sub)
        cache_size = 10
        sub = message_filters.Subscriber(status_topic, Image)
        # TODO: very strange
        cache = message_filters.Cache(sub, cache_size, allow_headerless=True)
        global_image_caches[image_topic] = cache

    thread = threading.Thread(target=_listener)
    thread.start()
