#!/usr/bin/env python
import json

import rospy
from sensor_msgs.msg import Image
import cv2
import threading
import rospy
import message_filters
from std_msgs.msg import String
from rospy import Duration

import os
import numpy as np
# Robotics Toolbox, and SWFIT for plotting
import roboticstoolbox as rtb
import math
from swift import Swift


class RoboticsToolboxSim():
    def __init__(self):
        path = os.path.dirname(__file__)
        path = os.path.join(path, "frcobot_description/fr5_robot.urdf")
        print('URDF path: ', path)

        # Roboticstoolbox Ploting
        # Create a FR5 instance and give it initial joint configurations.
        self.fr5 = rtb.Robot.URDF(path)

        qr = np.array([0, math.pi / 2, -math.pi / 2, 0, 0, 0])
        qz = np.zeros(6)
        qn = np.array([0, math.pi / 4, math.pi, 0, math.pi / 4, 0])
        self.fr5.addconfiguration_attr("q", np.array([0, 0, 0, 0, 0, 0]))
        self.fr5.addconfiguration_attr("qr", qr)
        self.fr5.addconfiguration_attr("qz", qz)
        self.fr5.addconfiguration_attr("qn", qn)

        # Make and instance of the Swift simulator and open it, and add robot arm to the environment
        self.real_env = Swift()
        self.real_env.launch(realtime=True)
        self.real_env.add(self.fr5, robot_alpha=True, collision_alpha=False)

    def step(self, action):
        self.fr5.q = action
        self.real_env.step(0.0667)


sim_env = RoboticsToolboxSim()


def callback(msg):
    data = msg.data
    try:
        action = json.loads(data)
        sim_env.step(np.radians(action[:6]))
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Error json loads: {e}")
    except Exception as e:
        rospy.logwarn(f"Error sim environment step: {e}")


def _listener():
    rospy.Subscriber('/sim_env/action', String, callback)
    rospy.spin()


rospy.init_node('sim_env', anonymous=True)
rospy.loginfo(f"Launch Sim Environment Rendering Node...")
thread = threading.Thread(target=_listener)
thread.start()
thread.join()
