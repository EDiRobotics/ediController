import json
import os
import time

import numpy as np
import rosbag
import rospy

rospy.init_node('rosbag_parser')

import json
import rosbag
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
import message_filters
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


def load_from_bag(file_name, config):
    try:
        bag = rosbag.Bag(file_name)
    except:
        return None
    # Define topics of interest
    status_topic = "/arm_status/all"
    sensor_topics = [topic for topic, _ in bag.get_type_and_topic_info()[1].items() if topic.startswith("/sensor/")]
    action_topic = "/env/step/action"
    idx_topic = "/env/step/idx"

    # Initialize data structures
    topics = {status_topic: [], action_topic: [], idx_topic: []}
    for sensor_topic in sensor_topics:
        topics[sensor_topic] = []

    # Dictionary to store idx timestamps
    idx_times = {}

    # Process messages from the bag file
    for topic, msg, t in bag.read_messages(topics=topics.keys()):
        if topic == idx_topic:
            idx_times[msg.data] = t
            topics[topic].append((msg.data, t))
        elif topic == action_topic:
            action_data = json.loads(msg.data)
            topics[topic].append((action_data, t))
        else:
            # For other topics, use the timestamp of the message
            topics[topic].append((msg, t))
    bag.close()

    # Process and pair actions with observations
    results = []
    for action_data, action_time in topics[action_topic]:
        idx = action_data["idx"]
        start_time = idx_times.get(idx - 1, None)
        end_time = action_time

        observations = {
            "status": [],
            "sensors": {sensor_topic: [] for sensor_topic in sensor_topics}
        }

        # Collect status and sensor data between idx timestamps
        for topic, messages in topics.items():
            if topic == status_topic or topic in sensor_topics:
                for msg, msg_time in messages:
                    if start_time is not None and msg_time > start_time:
                        # if start_time is not None and msg_time > start_time and msg_time <= end_time:
                        if topic == status_topic:
                            observations["status"].append(msg)
                        else:
                            observations["sensors"][topic].append(msg)
                        break

        results.append({
            "idx": idx,
            "action": action_data["action"],
            "obs": observations,
            "obs_timestamp": action_data["obs_timestamp"],
            "action_timestamp": action_data["timestamp"],
            "action_mode": action_data["mode"],
        })

    base_timestamp = results[0]["action_timestamp"]
    del results[0]
    del results[-1]

    for step, item in enumerate(results):
        idx = item["idx"]
        item["step"] = step
        observations = item["obs"]
        messages = observations["status"]
        if len(messages) == 0:
            rospy.logwarn(f"idx {idx} status is empty")
        for k, messages in observations["sensors"].items():
            if len(messages) == 0:
                rospy.logwarn(f"idx {idx} {k} is empty")
        del item["idx"]

    results = {"base_timestamp": base_timestamp, "records": results, "max_step": len(results)}
    results.update(config)
    return results


def find_files_with_extension(directory, extension):
    file_list = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                file_list.append(os.path.join(root, file))
    return file_list


file_dir = "./bag/"
extension = ".bag"
file_list = find_files_with_extension(file_dir, extension)

rospy.loginfo(f"Find RosBag file list: {file_list}")
all_results = []
for i, full_file_name in enumerate(file_list):

    file_name = os.path.basename(full_file_name)
    name = os.path.splitext(file_name)[0]
    folder_name = os.path.dirname(full_file_name)
    rospy.loginfo(f"[{i}] Loading {full_file_name}...")

    json_full_name = os.path.join(folder_name, "meta.json")
    if not os.path.exists(json_full_name):
        rospy.logerr(f"[{i}] meta json not found...")
        continue
    with open(json_full_name, 'r') as file:
        meta_data = json.load(file)
    results = load_from_bag(full_file_name, config=meta_data)
    all_results.append((full_file_name, results))

# TODO: Save data to LMDB Datasets

rospy.loginfo(f"----- Starting to replay -----")

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from backend.srv import StringService, StringServiceRequest, StringServiceResponse

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

    time.sleep(1)


original_state = rospy.get_param("/env/ctrl/switch", None)
rospy.set_param("/env/ctrl/switch", "replay")
for i, (full_file_name, results) in enumerate(all_results):
    rospy.loginfo(f"Start to replay {full_file_name}...")
    display_sensor_data(results)
rospy.set_param("/env/ctrl/switch", original_state)
rospy.loginfo(f"Finish all replay tasks, switch back to {original_state} state...")
