from edi_gym.edi_env import EdiEnv
import time

# ---- Test env basics ----
# env = EdiEnv()
# for i in range(5):
#     print(env.reset()["images"])
#     time.sleep(3)

# ---- Test env.step_with_action ----
base_joint = [23.0, -113.0, -102.0, -54.0, 90.0, -170.0]

while True:
    # TODO: Assign Joint Values
    joint = base_joint
    joint[0] += 0.05
    angle = 500
    action = joint + [angle]
    EdiEnv.step_with_action(action)
