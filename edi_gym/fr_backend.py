#!/usr/bin/env python
import json
import sys

sys.path.append(".")
import rospy
from std_msgs.msg import String, Int32
from std_srvs.srv import Trigger, TriggerResponse
from backend.srv import StringService, StringServiceRequest, StringServiceResponse
from hardware.arm_fr5 import fr5

try:
    robot_controller = fr5()
except Exception as e:
    rospy.logerr(f"Error occurs when launching Real Environment Backend: {e}")
    exit(0)


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
    global idx
    rospy.logdebug(f"[Backend] Respond to request: {request}")

    try:
        idx += 1
        publisher_idx.publish(idx)
        action_str = request.message
        action = json.loads(action_str)
        action_record = {"_action": action, "_timestamp": rospy.Time.now().to_time(), "_mode": 0}
        publisher_action.publish(json.dumps(action_record))
        info = step_with_action(action)
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


def rst_service(request):
    robot_controller.clear_errors()
    return TriggerResponse(
        success=True,
        message=""
    )


service_rst = rospy.Service('/env/reset', Trigger, rst_service)
service_demo = rospy.Service('/env/step/demo_action', StringService, handle_service)
service_policy = rospy.Service('/env/step/policy_action', StringService, handle_service)
rospy.spin()