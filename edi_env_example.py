from edi_env import EdiEnv
import time


# ---- Test env basics ----
import rospy
env = EdiEnv()
while not rospy.is_shutdown():
    print(env.reset()["images"])
    time.sleep(3)

# ---- Test env.step_with_action ----
# action = None
# EdiEnv.step_with_action(action)
