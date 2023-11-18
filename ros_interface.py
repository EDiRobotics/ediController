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

global_image = {}
global_status = {}
global_image_caches = {}
global_status_caches = {}
state_cache = None

image_topics = []
cv_bridge = CvBridge()

def _listener():
    global global_status_caches, global_image_caches
    status_topic = "/arm_status"
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
        cache = message_filters.Cache(sub, cache_size)
        global_image_caches[image_topic] = cache

    rospy.spin()


# def _display_image():
#     global global_image
#     while not rospy.is_shutdown():
#         print(status)
#         for k, v in global_image:
#             cv2.imshow(k, v)
#             cv2.waitKey(1)


rospy.init_node('ros_interface', anonymous=True)
action_topic_name = "/arm_control"
action_publisher = rospy.Publisher(action_topic_name, String, queue_size=10)

thread = threading.Thread(target=_listener)
thread.start()

# _display_image()
