#!/usr/bin/python3
import json
import os
import datetime
import queue
import signal
import subprocess
import threading
import time

import rospy
from std_msgs.msg import Int8
from std_srvs.srv import Trigger, TriggerResponse
from data_collection.bag_loader import record

datetime_start = datetime.datetime.now()
datetime_start = datetime_start.strftime("%Y%m%d%H%M%S")
meta = {"datetime": datetime_start}

rospy.init_node('record_bags')
lmdb_save_path = "dataset"
rospy.loginfo(f"[bag] Init records node, lmdb save path is {lmdb_save_path}")

current_process = None
current_bag_full_path = None
i = 0

save_queue = queue.Queue()


def start_record_service(req):
    global current_process, current_bag_full_path, i
    if current_process is None:
        i += 1
        bag_name = f'all_topics'
        now = datetime.datetime.now()
        now = now.strftime("%Y%m%d_%H%M%S")
        dir_name = f'./bag/{now}/'
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        current_bag_full_path = os.path.join(dir_name, bag_name)
        rospy.set_param('/record/ctrl/save_directory', dir_name)
        current_process = start_record(current_bag_full_path)
        return TriggerResponse(success=True, message="Recording started.")
    else:
        return TriggerResponse(success=False, message="Recording is already running.")


def end_record_service(req):
    global current_process, current_bag_full_path, i
    if current_process is not None:
        end_record(current_process, current_bag_full_path)
        current_process = None
        return TriggerResponse(success=True, message="Recording stopped.")
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


def end_record(process: subprocess.Popen, bag_full_path: str):
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    process.wait()
    process.terminate()
    rospy.set_param('/record/ctrl/recording', False)
    rospy.loginfo(f"\n--- End record {bag_full_path} ---\n")
    save_queue.put(bag_full_path)


# start_service = rospy.Service('/record/ctrl/start_record_srv', Trigger, start_record_service)
# end_service = rospy.Service('/record/ctrl/end_record_srv', Trigger, end_record_service)
#
# rospy.spin()


def convert():
    while not rospy.is_shutdown():
        if save_queue.empty():
            time.sleep(0.5)
            continue
        bag_full_path = save_queue.get()
        record(bag_full_path, lmdb_save_path=lmdb_save_path)


convert_thread = threading.Thread(target=convert)
convert_thread.daemon = True
convert_thread.start()

if __name__ == "__main__":
    start_record_service(None)
    time.sleep(8)
    end_record_service(None)
