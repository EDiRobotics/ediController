from edi_gym.edi_env import EdiEnv
import time


# ---- Test env basics ----
env = EdiEnv()
for i in range(5):
    print(env.reset()["images"])
    time.sleep(3)

# ---- Test env.step_with_action ----
# action = None
# EdiEnv.step_with_action(action)
