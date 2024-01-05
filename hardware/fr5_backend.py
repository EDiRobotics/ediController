#!/usr/bin/env python
import json
import queue
import sys
import threading
import time
import traceback

sys.path.append(".")
import rospy
from std_msgs.msg import String, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse
from backend.srv import StringService, StringServiceRequest, StringServiceResponse
from hardware.arm_fr5 import fr5

try:
    robot_controller = fr5()
except Exception as e:
    rospy.logerr(f"Error occurs when launching Real Environment Backend: {e}")
    exit(0)

control_freq = 50
control_t = 1 / control_freq
idx = 0
last_action = time.time()
obs_time = rospy.Time.now()


def step_with_action(action):
    """
    :param action: 7 length list. The first 6 item contain the 6 joint angle,
    the last item contains the gripper proportion.
    :return: a dict containing some information
    """
    global last_action
    action = [float(a) for a in action]
    joint = action[:6]
    gripper = action[-1]
    publisher_gripper.publish(gripper)
    # real env step
    if time.time() - last_action > 1:
        robot_controller.reset_first()
    last_action = time.time()
    retJ = robot_controller.move_joint(joint)
    retG = robot_controller.set_gripper(gripper)
    err, errors = robot_controller.detect_errors()

    err = err or retJ or retG
    if err is not None:
        err = int(err)
    else:
        rospy.logwarn(f"The err is None by accident, it should be an integer. Currently retJ: {retJ}, retG: {retG}")
        err = 0
    errors.append(robot_controller.lookup_error(retJ))
    errors.append(robot_controller.lookup_error(retG))

    step_action_info = dict()
    step_action_info["error"] = err
    step_action_info["error_details"] = errors
    return step_action_info


rospy.init_node('backend', anonymous=True)
rospy.loginfo(f"Launch Real Environment Backend...")

topic_name = "/arm_status/gripper_pos"
publisher_gripper = rospy.Publisher(topic_name, Float32, queue_size=5)
topic_name = "/env/step/idx"
publisher_idx = rospy.Publisher(topic_name, Int32, queue_size=100)
topic_name = "/env/step/action"
publisher_action = rospy.Publisher(topic_name, String, queue_size=100)
topic_name = "/env/step/info"
publisher_info = rospy.Publisher(topic_name, String, queue_size=100)


class TrajectoryOptimizer:
    def __init__(self, optimize_interval=3):
        self.optimize_interval = optimize_interval
        self.action_queue = queue.Queue()

    def optimize_traj(self):
        while not rospy.is_shutdown():
            time.sleep(self.optimize_interval)

    def add_action(self, action):
        self.action_queue.put(action)

    def get_action(self, time):
        action = []
        return action


traj_opt = TrajectoryOptimizer()


class ArmControlThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while not rospy.is_shutdown():
            time_now = rospy.Time.now()
            if not rospy.Time.now() >= obs_time + rospy.Duration(secs=0.1):
                time.sleep((obs_time + rospy.Duration(secs=0.1) - rospy.Time.now()).to_sec())
            else:
                rospy.logwarn("Missing desired control frequency...")
            action = traj_opt.get_action(time_now)
            try:
                step_with_action(action)
            except Exception as e:
                traceback.print_exc()
                rospy.logwarn(f"Error happened: {e}")


control_thread = ArmControlThread()
control_thread.start()


def handle_service(request: StringServiceRequest):
    global idx, last_switch_state, obs_time
    info = {"error": 0}
    try:
        action_str = request.message
        action = json.loads(action_str)
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Error json loads: {e}")
        error_msg = json.dumps({"error": "JSON decode error"})
        return StringServiceResponse(success=False, message=error_msg)

    if not valid(action):
        rospy.logwarn(f"Action {action} is not valid...")
        info["error"] = 4
        error_msg = json.dumps(info)
        return StringServiceResponse(success=False, message=error_msg)
    idx += 1
    publisher_idx.publish(idx)
    action_time = rospy.Time.now()
    action_record = {"idx": idx, "action": action,
                     "timestamp": action_time.to_time(),
                     "obs_timestamp": obs_time.to_time(),
                     "mode": last_switch_state}
    publisher_action.publish(json.dumps(action_record))
    traj_opt.add_action(action)
    obs_time = rospy.Time.now()
    idx += 1
    publisher_idx.publish(idx)
    info_str = json.dumps(info)
    return StringServiceResponse(
        success=True,
        message=info_str
    )


def handle_service_demo(request):
    if rospy.get_param("/env/ctrl/demo", False):
        return handle_service(request)
    else:
        return StringServiceResponse(success=False, message="Not Allowed")


def handle_service_policy(request):
    if rospy.get_param("/env/ctrl/policy", False):
        return handle_service(request)
    else:
        return StringServiceResponse(success=False, message="Not Allowed")


def handle_service_replay(request):
    if rospy.get_param("/env/ctrl/replay", False):
        return handle_service(request)
    else:
        return StringServiceResponse(success=False, message="Not Allowed")


last_switch_state = "policy"
rospy.set_param("/env/ctrl/switch", last_switch_state)
rospy.set_param("/env/ctrl/policy", True)
rospy.set_param("/env/ctrl/demo", False)
rospy.set_param("/env/ctrl/replay", False)


def switch(event):
    global last_switch_state
    current_switch_state = rospy.get_param("/env/ctrl/switch")
    if current_switch_state not in ["policy", "demo", "replay"]:
        current_switch_state = last_switch_state
        rospy.set_param("/env/ctrl/switch", current_switch_state)

    if current_switch_state != last_switch_state:
        last_switch_state = current_switch_state

        robot_controller.reset_first()
        new_policy_state = (current_switch_state == "policy")
        new_demo_state = (current_switch_state == "demo")
        new_replay_state = (current_switch_state == "replay")

        rospy.set_param("/env/ctrl/policy", new_policy_state)
        rospy.set_param("/env/ctrl/demo", new_demo_state)
        rospy.set_param("/env/ctrl/replay", new_replay_state)
        robot_controller.reset_first()

        if new_policy_state:
            rospy.loginfo("Policy is now enabled.")
        else:
            rospy.loginfo("Policy is now disabled.")

        if new_demo_state:
            rospy.loginfo("Demo is now enabled.")
        else:
            rospy.loginfo("Demo is now disabled.")

        if new_replay_state:
            rospy.loginfo("Replay is now enabled.")
        else:
            rospy.loginfo("Replay is now disabled.")

    assert (int(rospy.get_param("/env/ctrl/policy")) +
            int(rospy.get_param("/env/ctrl/demo")) +
            int(rospy.get_param("/env/ctrl/replay")) <= 1
            )


def rst_service(request):
    try:
        robot_controller.clear_errors()
        robot_controller.reset_first()
        return TriggerResponse(
            success=True,
            message=""
        )
    except Exception as e:
        return TriggerResponse(
            success=False,
            message=f"{str(e)}"
        )


service_rst = rospy.Service('/env/reset_srv', Trigger, rst_service)
service_replay = rospy.Service('/env/step/replay_action_srv', StringService, handle_service_replay)
service_demo = rospy.Service('/env/step/demo_action_srv', StringService, handle_service_demo)
service_policy = rospy.Service('/env/step/policy_action_srv', StringService, handle_service_policy)
timer = rospy.Timer(rospy.Duration(nsecs=100), switch)
rospy.spin()
