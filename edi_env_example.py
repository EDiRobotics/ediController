from edi_gym.edi_env import EdiEnv
import time
import cv2

base_joint = [23.0, -113.0, -102.0, -54.0, 90.0, -170.0]
joint = base_joint
joint[0] += 0.05
angle = 500
action = joint + [angle]

# ---- Test env basics ----
# """
# Only available when rospy is found.
# """

env = EdiEnv()
max_step = 200


def eval(model, env):
    model.eval()
    obs = env.reset()
    for step in range(max_step):
        action = model(obs)
        obs, _, _, info = env.step(action)
        for camera_name, image in obs["sensors"].items():
            cv2.imshow(camera_name, image)
        cv2.waitKey(1)

# ---- Test env.step_with_action only ----
# while True:
#     joint[0] += 0.05
#     angle = 500
#     action = joint + [angle]
#     EdiEnv.step_with_action(action)
