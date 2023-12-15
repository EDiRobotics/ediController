import json
import threading
import traceback

import numpy as np
import time
import rospy
from std_msgs.msg import String, Int32
import sys
sys.path.append(".")
from edi_gym.edi_env import EdiEnv

env = EdiEnv()
_ = env.reset()

idx = 0
rospy.loginfo(f"Launch Data Collection...")

topic_name = "/env/step_dix"
publisher_idx = rospy.Publisher(topic_name, Int32, queue_size=10)
publisher_info = rospy.Publisher(topic_name, String, queue_size=10)


def callback(msg):
    global idx
    data = msg.data
    try:
        action = json.loads(data)
        idx += 1
        publisher_idx.publish(idx)
        _, _, _, info = env.step(action)
        info_str = json.dumps(info)
        publisher_idx.publish(info_str)
        idx += 1
        publisher_idx.publish(idx)
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Error json loads: {e}")
    except Exception as e:
        traceback.print_exc()
        rospy.logwarn(f"[Collect] Error happened: {e}")


rospy.Subscriber('/env/action/demo_action', String, callback, queue_size=1)
rospy.Subscriber('/env/action/policy_action', String, callback, queue_size=1)
rospy.spin()
