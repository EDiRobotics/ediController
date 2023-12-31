import argparse
import datetime
import json
import sys
import os
import time
import traceback
import numpy as np
import rosbag
import rospy
import json
import rosbag
from std_msgs.msg import String, Int32
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy

sys.path.append(".")
from data_collection.lmdb_interface import save_to_lmdb, generate_gif

bridge = CvBridge()


def load_from_bag(file_name, config=None):
    if config is None:
        config = {}
    dir_name, _ = os.path.split(file_name)
    params_file_name = os.path.join(dir_name, 'params.json')
    params = {}
    if os.path.exists(params_file_name):
        try:
            with open(params_file_name, 'r') as f:
                params = json.load(f)
        except:
            rospy.logwarn(f"[loader] Error decoding {params_file_name}, use default...")
    try:
        bag = rosbag.Bag(file_name)
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
    except:
        traceback.print_exc()
        rospy.logerr(f"[loader] Reading topics from {params_file_name} rosbag file.")

        return None

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
                            status = json.loads(msg.data)
                            observations["status"].append(status)
                        else:
                            msg: Image
                            img = bridge.imgmsg_to_cv2(msg, "bgr8")
                            observations["sensors"][topic].append(img)
                        break

        results.append({
            "idx": idx,
            "action": action_data["action"],
            "obs": observations,
            "obs_timestamp": action_data["obs_timestamp"],
            "action_timestamp": action_data["timestamp"],
            "action_mode": action_data["mode"],
        })

    if len(results) <= 2:
        rospy.logerr(f"[loader] Failed to process {params_file_name}, len is {len(results)}...")
        return None
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
            observations["status"] = None
        else:
            observations["status"] = messages[0]
        for k, messages in observations["sensors"].items():
            if len(messages) == 0:
                rospy.logwarn(f"idx {idx} {k} is empty")
                observations["sensors"][k] = None
            else:
                observations["sensors"][k] = messages[0]
        del item["idx"]
    instruct = ""
    instruct_param = "/env/info/instruct"
    if instruct_param in params:
        instruct = params[instruct_param]

    duration = float(results[-1]["action_timestamp"]) - float(results[0]["obs_timestamp"])
    results = {"base_timestamp": base_timestamp, "records": results, "duration": duration,
               "max_step": len(results), "instruct": instruct, "params": params}
    results.update(config)
    return results


def find_files_with_extension(directory, extension):
    file_list = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                file_list.append(os.path.join(root, file))
    return file_list


def record(input_path, lmdb_save_path=None, delete_bag=False, cover_exist=False, gif=True):
    if not os.path.exists(input_path):
        rospy.loginfo(f"Input path {input_path} doesn't exist, exiting...")
        exit(1)
    if os.path.isfile(input_path) and input_path.endswith(".bag"):
        file_list = [input_path]
    elif os.path.isdir(input_path):
        file_dir = input_path
        extension = ".bag"
        file_list = find_files_with_extension(file_dir, extension)
    else:
        raise NotImplementedError

    rospy.loginfo(f"[loader] Find rosbag file list: {file_list}")
    all_results = []
    for i, full_file_name in enumerate(file_list):
        file_name = os.path.basename(full_file_name)
        name = os.path.splitext(file_name)[0]
        folder_name = os.path.dirname(full_file_name)
        rospy.loginfo(f"[loader] <{i}/{len(file_list)}> Loading {full_file_name}...")
        json_full_name = os.path.join(folder_name, "meta.json")
        if not os.path.exists(json_full_name):
            rospy.logwarn(f"[loader] <{i}/{len(file_list)}> meta.json missed for {full_file_name}...")
            datetime_start = datetime.datetime.now()
            datetime_start = datetime_start.strftime("%Y%m%d%H%M%S")
            meta_data = {"episode": f"missed_{datetime_start}"}
        else:
            with open(json_full_name, 'r') as file:
                meta_data = json.load(file)

        if "save_path" not in meta_data or cover_exist:
            results = load_from_bag(full_file_name, config=meta_data)
            if results is None:
                rospy.logerr(f"Error load from bag {full_file_name}, results is None!")
                continue
            if lmdb_save_path is not None:
                success = save_to_lmdb(results, lmdb_save_path, gif=gif)
                if not success:
                    rospy.logerr(f"Error occurs when dumping {full_file_name}!")
                    continue

                absolute_lmdb_save_path = os.path.abspath(lmdb_save_path)
                meta_data["save_path"] = absolute_lmdb_save_path
                meta_data["duration"] = results["duration"]
                with open(json_full_name, 'w') as file:
                    json.dump(meta_data, file, indent=4)

                if delete_bag:
                    try:
                        os.remove(full_file_name)
                        rospy.loginfo(f"[loader] <{i}/{len(file_list)}> Deleted {full_file_name}")
                    except OSError as e:
                        rospy.logerr(f"[loader] <{i}/{len(file_list)}> Error deleting {full_file_name}: {e.strerror}")

            all_results.append((full_file_name, results))
        else:
            rospy.logwarn(f"[loader] <{i}/{len(file_list)}> Pass {full_file_name} because save_path exists...")

    rospy.loginfo(f"[loader] Record {input_path} finished!")

    return all_results


if __name__ == "__main__":
    """
    Test load rosbag and replay
    """
    rospy.init_node('rosbag_parser')

    parser = argparse.ArgumentParser(description='Bag Loader To LMDB Dataset',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--path', type=str, default="./dataset/bag/",
                        help='Path can be a rosbag file or a directory (recursively search).')
    parser.add_argument('--delete_bag', default=False, action='store_true',
                        help='Whether to delete the original rosbag file, default is False')
    parser.add_argument('--display_image', '-i', action='store_true',
                        help='Replay image with cv2')
    parser.add_argument('--display_action', '-a', action='store_true',
                        help='Replay action')
    args = parser.parse_args()
    display_image = args.display_image
    display_action = args.display_action

    input_path: str = args.path
    delete_bag: bool = args.delete_bag

    file_dir = input_path
    extension = ".bag"
    file_list = find_files_with_extension(file_dir, extension)
    all_results = []
    for file_path in file_list:
        with open(os.path.join(os.path.dirname(file_path), "meta.json"), 'r') as file:
            d = json.load(file)
            episode = d["episode"]
        lmdb_save_path = f"dataset/train_{episode}_lmdb"
        # lmdb_save_path = f"dataset/train_nav_lmdb"
        all_results += record(file_path, lmdb_save_path=lmdb_save_path, delete_bag=delete_bag, cover_exist=True)

    rospy.loginfo(f"----- Starting to replay -----")
    from data_collection.replay import display_sensor_data

    for i, (full_file_name, results) in enumerate(all_results):
        rospy.loginfo(f"Start to replay {full_file_name}...")
        display_sensor_data(results, display_image=display_image, display_action=display_action)

    rospy.loginfo(f"Finish all replay tasks...")
