#!/usr/bin/python3
import json
import os
import datetime
import queue
import shutil
import signal
import subprocess
import threading
import time
import sys
import traceback

import rospy
import psutil
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest

sys.path.append(".")
from data_collection.bag_loader import record

lmdb_save_path_is_fixed = True
delete_bag = True

rospy.init_node('record_bags')
rospy.loginfo(f"[record] Initialize records node.")

heartbeat_timestamps = {None: rospy.get_time()}
status_timestamps = rospy.get_time()
backend_timestamps = rospy.get_time()
latest_client = None

bag_save_base = f"dataset/bag"
rospy.loginfo(f"[record] Rosbag save path is set to {bag_save_base}.")
if not os.path.exists(bag_save_base):
    os.makedirs(bag_save_base)

datetime_start = datetime.datetime.now()
datetime_start = datetime_start.strftime("%Y%m%d%H%M%S")
meta = {"datetime": datetime_start}

lmdb_save_path = f"dataset/train_{datetime_start}_lmdb"
if lmdb_save_path_is_fixed:
    rospy.set_param('/record/ctrl/fixed_lmdb', True)
    rospy.loginfo(f"[record] LMDB save path is fixed, set to {lmdb_save_path}.")
else:
    rospy.set_param('/record/ctrl/fixed_lmdb', False)
    rospy.loginfo(f"[record] LMDB save path is not fixed.")

current_process = None
current_bag_full_path = None
i = 0

save_queue = queue.Queue()


def start_record_service(req):
    global current_process, current_bag_full_path, i
    current_time = rospy.get_time()
    heartbeat_checkup = current_time - heartbeat_timestamps[latest_client] > 3
    status_checkup = current_time - status_timestamps > 3
    backend_heartbeat_checkup = current_time - backend_timestamps > 1
    if heartbeat_checkup or status_checkup or backend_heartbeat_checkup:
        return TriggerResponse(success=False, message="Did not receive status or heartbeat, can not start recording.")
    if current_process is None:
        i += 1
        bag_name = f'all_topics.bag'
        now = datetime.datetime.now()
        now = now.strftime("%Y%m%d_%H%M%S")
        dir_name = os.path.join(bag_save_base, now)
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        current_bag_full_path = os.path.join(dir_name, bag_name)
        rospy.set_param('/record/ctrl/save_directory', dir_name)
        current_process = start_record(current_bag_full_path)
        return TriggerResponse(success=True, message=current_bag_full_path)
    else:
        return TriggerResponse(success=False, message="Recording is already running.")


def end_record_service(req):
    global current_process, current_bag_full_path, i
    if current_process is not None:
        end_record(current_process, current_bag_full_path)
        current_process = None
        return TriggerResponse(success=True, message=os.path.abspath(current_bag_full_path))
    else:
        return TriggerResponse(success=False, message="No recording is currently running.")


def start_record(bag_full_path: str) -> subprocess.Popen:
    dir_name, _ = os.path.split(bag_full_path)
    all_params = rospy.get_param_names()
    d = {k: str(rospy.get_param(k)) for k in all_params}
    with open(os.path.join(dir_name, 'params.json'), 'w') as f:
        json.dump(d, f)
    meta = {"episode": int(datetime_start) * 1000 + i}
    with open(os.path.join(dir_name, 'meta.json'), 'w') as f:
        json.dump(meta, f)
    rospy.set_param('/record/ctrl/recording', True)
    command = "rosbag record -a -O {} ".format(bag_full_path)
    rospy.loginfo(f"\n--- Start record {bag_full_path} ---\n")
    process = subprocess.Popen(command, shell=True)
    return process


def terminate_process_and_children(p):
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()
    p.terminate()


def end_record(process: subprocess.Popen, bag_full_path: str):
    global save_queue
    rospy.loginfo(f"\n--- Killing record of {bag_full_path} ---\n")
    dir_name, _ = os.path.split(bag_full_path)
    all_params = rospy.get_param_names()
    d = {k: str(rospy.get_param(k)) for k in all_params}
    with open(os.path.join(dir_name, 'params_end.json'), 'w') as f:
        json.dump(d, f)
    terminate_process_and_children(process)
    # process.send_signal(signal.SIGINT)
    rospy.set_param('/record/ctrl/recording', False)
    rospy.loginfo(f"\n--- End record {bag_full_path} ---\n")
    save_queue.put(bag_full_path)
    time.sleep(1)


def convert():
    while not rospy.is_shutdown():
        global save_queue, lmdb_save_path
        if save_queue.empty():
            time.sleep(.1)
            # rospy.loginfo(f"[record] Queue is empty, waiting for rosbag.")
            continue
        bag_full_path = save_queue.get()
        dir_name, _ = os.path.split(bag_full_path)

        datetime_now = datetime.datetime.now()
        datetime_now = datetime_now.strftime("%Y%m%d%H%M%S")
        if not lmdb_save_path_is_fixed:
            lmdb_save_path = f"dataset/train_{datetime_now}_lmdb"
        rospy.loginfo(f"[record] Get {bag_full_path}, start saving to {lmdb_save_path}...")
        try:
            time.sleep(1)
            record(bag_full_path, lmdb_save_path=lmdb_save_path, delete_bag=delete_bag)
        except Exception as e:
            traceback.print_exc()
            rospy.logerr(f"[record] Saving {bag_full_path} error: {e}...")


convert_thread = threading.Thread(target=convert)
convert_thread.start()


def client_heartbeat_callback(msg):
    global latest_client, heartbeat_timestamps
    latest_client = msg.data
    heartbeat_timestamps[latest_client] = rospy.get_time()
    rospy.loginfo(f"Heartbeat received from client {latest_client}")


def status_callback(msg):
    global status_timestamps
    status_timestamps = rospy.get_time()


def backend_callback(msg):
    global backend_timestamps
    backend_timestamps = rospy.get_time()


def check_heartbeat():
    global latest_client, heartbeat_timestamps, status_timestamps, current_process, current_bag_full_path
    while not rospy.is_shutdown():
        if current_process is not None and latest_client in heartbeat_timestamps:
            current_time = rospy.get_time()
            heartbeat_checkup = current_time - heartbeat_timestamps[latest_client] > 3
            status_checkup = current_time - status_timestamps > 3
            backend_heartbeat_checkup = current_time - backend_timestamps > 1
            if heartbeat_checkup or status_checkup or backend_heartbeat_checkup:
                rospy.logwarn(f"Heartbeat timeout for client {latest_client}. Ending recording.")
                end_record_service_response = end_record_service(TriggerRequest())
                if end_record_service_response.success:
                    try:
                        shutil.rmtree(os.path.dirname(current_bag_full_path))
                        rospy.loginfo(f"Successfully deleted directory {os.path.dirname(current_bag_full_path)}")
                    except OSError as e:
                        rospy.logerr(f"Error deleting directory: {e}")

        rospy.sleep(5)


heartbeat_thread = threading.Thread(target=check_heartbeat)
heartbeat_thread.start()

client_heartbeat_subscriber = rospy.Subscriber("/record/ctrl/client", String, client_heartbeat_callback)
status_subscriber = rospy.Subscriber("/arm_status/all", String, status_callback)
backend_subscriber = rospy.Subscriber("/env/ctrl/backend_heartbeat", String, backend_callback)

start_service = rospy.Service('/record/ctrl/start_record_srv', Trigger, start_record_service)
end_service = rospy.Service('/record/ctrl/end_record_srv', Trigger, end_record_service)
rospy.spin()
convert_thread.join()
heartbeat_thread.join()
