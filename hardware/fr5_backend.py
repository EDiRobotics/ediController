#!/usr/bin/env python
import json
import sys

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

topic_name = "/arm_status/gripper_pos"
publisher_gripper = rospy.Publisher(topic_name, Float32, queue_size=5)


def step_with_action(action):
    """
    :param action: 7 length list. The first 6 item contain the 6 joint angle,
    the last item contains the gripper proportion.
    :return: a dict containing some information
    """
    # action = cls._action_chunk(action)
    action = [float(a) for a in action]
    joint = action[:6]
    gripper = action[-1]
    publisher_gripper.publish(gripper)
    # real env step
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

topic_name = "/env/step/idx"
publisher_idx = rospy.Publisher(topic_name, Int32, queue_size=10)
topic_name = "/env/step/action"
publisher_action = rospy.Publisher(topic_name, String, queue_size=10)
topic_name = "/env/step/info"
publisher_info = rospy.Publisher(topic_name, String, queue_size=10)

idx = 0


def handle_service(request: StringServiceRequest):
    global idx, last_switch_state
    try:
        idx += 1
        publisher_idx.publish(idx)
        action_str = request.message
        action = json.loads(action_str)
        action_record = {"idx": idx, "action": action, "timestamp": rospy.Time.now().to_time(),
                         "mode": last_switch_state}
        publisher_action.publish(json.dumps(action_record))
        info = step_with_action(action)
        idx += 1
        publisher_idx.publish(idx)
        info_str = json.dumps(info)
        publisher_info.publish(info_str)
        return StringServiceResponse(
            success=True,
            message=info_str
        )
    except json.JSONDecodeError as e:
        rospy.logwarn(f"Error json loads: {e}")
        return StringServiceResponse(success=False, message="JSON decode error")
    except Exception as e:
        rospy.logwarn(f"Error happened: {e}")
        return StringServiceResponse(success=False, message="Exception occurred")


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


last_switch_state = "policy"
rospy.set_param("/env/ctrl/switch", last_switch_state)
rospy.set_param("/env/ctrl/policy", True)
rospy.set_param("/env/ctrl/demo", False)


def switch(event):
    global last_switch_state
    current_switch_state = rospy.get_param("/env/ctrl/switch")
    if current_switch_state not in ["policy", "demo"]:
        current_switch_state = last_switch_state
        rospy.set_param("/env/ctrl/switch", current_switch_state)

    if current_switch_state != last_switch_state:
        last_switch_state = current_switch_state

        current_policy_state = rospy.get_param("/env/ctrl/policy")
        current_demo_state = rospy.get_param("/env/ctrl/demo")

        robot_controller.reset_first()
        new_policy_state = not current_policy_state
        new_demo_state = not current_demo_state

        rospy.set_param("/env/ctrl/policy", new_policy_state)
        rospy.set_param("/env/ctrl/demo", new_demo_state)
        robot_controller.reset_first()

        if new_policy_state:
            rospy.loginfo("Policy is now enabled.")
        else:
            rospy.loginfo("Policy is now disabled.")

        if new_demo_state:
            rospy.loginfo("Demo is now enabled.")
        else:
            rospy.loginfo("Demo is now disabled.")

    assert rospy.get_param("/env/ctrl/policy") != rospy.get_param("/env/ctrl/demo")


def rst_service(request):
    robot_controller.clear_errors()
    return TriggerResponse(
        success=True,
        message=""
    )


service_rst = rospy.Service('/env/reset', Trigger, rst_service)
service_demo = rospy.Service('/env/step/demo_action', StringService, handle_service_demo)
service_policy = rospy.Service('/env/step/policy_action', StringService, handle_service_policy)
timer = rospy.Timer(rospy.Duration(nsecs=10), switch)
rospy.spin()
