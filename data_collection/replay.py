import time
import numpy as np
import rosbag
import rospy
import json
import rosbag
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
import message_filters
import rospy
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from backend.srv import StringService, StringServiceRequest, StringServiceResponse
import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

service = '/env/step/replay_action_srv'
rospy.wait_for_service(service)
action_service = rospy.ServiceProxy(service, StringService)
rospy.loginfo(f"Server {service} is ready...")


def execute_action(action):
    if isinstance(action, np.ndarray):
        action = action.tolist()
    action_json = json.dumps(action)
    request = StringServiceRequest(action_json)
    rospy.logdebug(f"Request action: {action_json}")
    response = action_service(request)
    if not response.success:
        rospy.logerr(f"Request action return errors: {response.message}")


def display_sensor_data(results):
    original_state = rospy.get_param("/env/ctrl/switch", None)
    rospy.set_param("/env/ctrl/switch", "replay")

    base_timestamp = results["base_timestamp"]
    base_timestamp = rospy.Time(base_timestamp)
    time_offset = rospy.Time.now() - base_timestamp

    for i, result in enumerate(results["records"]):
        observations = result['obs']
        obs_timestamp = result['obs_timestamp']
        obs_timestamp = rospy.Time(obs_timestamp)
        adjusted_obs_timestamp = obs_timestamp + time_offset

        if rospy.Time.now() < adjusted_obs_timestamp:
            rospy.sleep(adjusted_obs_timestamp - rospy.Time.now())

        for sensor_topic, sensor_msgs in observations['sensors'].items():
            for sensor_msg in sensor_msgs:
                try:
                    cv_image = bridge.imgmsg_to_cv2(sensor_msg, "bgr8")
                except CvBridgeError as e:
                    rospy.logerr(e)
                    break
                cv2.imshow(sensor_topic, cv_image)
                cv2.waitKey(1)

        action_timestamp = result['action_timestamp']
        action_timestamp = rospy.Time(action_timestamp)

        action = result['action']
        adjusted_act_timestamp = action_timestamp + time_offset
        if rospy.Time.now() < adjusted_act_timestamp:
            rospy.sleep(adjusted_act_timestamp - rospy.Time.now())
        execute_action(action)

    cv2.destroyAllWindows()

    time.sleep(0.5)
    rospy.set_param("/env/ctrl/switch", original_state)