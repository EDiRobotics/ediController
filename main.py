#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String


def execute_action(action):
    # TODO: action chunk, simply execute first action...
    pass


def action_callback(data):
    action_data = json.loads(data.data)
    action = action_data['action']
    # Process the action here
    print("Received action:", action)


def listener():
    rospy.init_node('arm_controller', anonymous=True)
    rospy.Subscriber("/arm_control", String, action_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
