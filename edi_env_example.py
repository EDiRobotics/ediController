from edi_gym.edi_env import EdiEnv
import time

# ---- Test env basics ----
# env = EdiEnv()
# for i in range(5):
#     print(env.reset()["images"])
#     time.sleep(3)

# ---- Test env.step_with_action ----
while True:
    # TODO: Assign Joint Values
    joint = [23.0, -113.0, -102.0, -54.0, 90.0, -170.0]
    angle = 500
    action = joint + [angle]
    EdiEnv.step_with_action(action)
