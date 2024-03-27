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
from data_collection.bag_loader import record

sys.path.append(".")

rospy.init_node('record_bags')
rospy.loginfo(f"[record] Initialize records node.")

bag_save_base = f"dataset/bag"
lmdb_save_base = f"dataset/train_lmdb"
lmdb_save_path_is_fixed = True


def terminate_process_and_children(p):
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()
    p.terminate()


class Record:

    def __init__(self, bag_path, client=None, meta=None):
        self.bag_path = bag_path
        self.process = None
        if meta is None:
            meta = dict()
        self.meta = meta
        self.client = client
        self._record = 0

    def start(self):
        dir_name, _ = os.path.split(self.bag_path)
        meta = self.meta
        if isinstance(meta, dict):
            meta["recording"] = True
            with open(os.path.join(dir_name, 'meta.json'), 'w') as f:
                json.dump(meta, f)
        all_params = rospy.get_param_names()
        d = {k: str(rospy.get_param(k)) for k in all_params}
        with open(os.path.join(dir_name, 'params.json'), 'w') as f:
            json.dump(d, f)
        rospy.set_param('/record/ctrl/recording', True)
        command = "rosbag record -a -O {} ".format(self.bag_path)
        self.process = subprocess.Popen(command, shell=True)
        self._record = 1

    def end(self):
        if self._record != 1:
            return
        dir_name, _ = os.path.split(self.bag_path)
        meta = self.meta
        if isinstance(meta, dict):
            meta["recording"] = False
            with open(os.path.join(dir_name, 'meta.json'), 'w') as f:
                json.dump(meta, f)
        all_params = rospy.get_param_names()
        d = {k: str(rospy.get_param(k)) for k in all_params}
        with open(os.path.join(dir_name, 'params_end.json'), 'w') as f:
            json.dump(d, f)
        try:
            terminate_process_and_children(self.process)
        except:
            rospy.logwarn("Error ending the process, perhaps it has already been terminated.")
        self._record = -1

    @property
    def full_path(self):
        return os.path.abspath(self.bag_path)

    def set_meta(self, d):
        self.meta.update(d)


class rosbagRecorder:
    i = 1

    def __init__(self, base):

        self.base = base
        if not os.path.exists(self.base):
            os.makedirs(self.base)
        rospy.loginfo(f"[record] rosbag save path is set to {self.base}.")

        self.datetime_start = datetime.datetime.now()
        self.datetime_start = self.datetime_start.strftime("%Y%m%d%H%M%S")

        self.record = None
        self.client = None
        self.heartbeat_timestamps = {None: rospy.get_time()}
        self.env_timestamps = None
        self.mutex = threading.Lock()

        self.heartbeat_thread = threading.Thread(target=self.check_heartbeat)
        self.heartbeat_thread.start()

    def check_record_availability(self, client):
        current_time = rospy.get_time()
        heartbeat_checkup = current_time - self.heartbeat_timestamps[client] <= 3
        env_checkup = True
        # env_checkup = current_time - self.env_timestamps <= 3
        availability = heartbeat_checkup and env_checkup
        if not heartbeat_checkup:
            rospy.logwarn("Heartbeat not available!")
        if not env_checkup:
            rospy.logwarn("Environment is down!")
        return availability

    def check_heartbeat(self):
        while not rospy.is_shutdown():
            self.mutex.acquire()
            rm_path = None
            if self.record is not None:
                if not self.check_record_availability(self.record.client):
                    rospy.logwarn(f"Heartbeat timeout for client {self.record.client}. Ending recording.")
                    self.record.end()
                    rospy.set_param('/record/ctrl/recording', False)
                    rm_path = os.path.dirname(self.record.bag_path)
                    self.record = None
            self.mutex.release()
            if rm_path:
                try:
                    # TODO: remove this directory
                    shutil.rmtree(rm_path)
                    rospy.logwarn(f"\n--- Kill record, deleting {rm_path} ---")
                except OSError as e:
                    rospy.logerr(f"\n--- Kill record, Error deleting {rm_path} ---")
            rospy.sleep(0.1)

    def start_record_service(self, req):
        client = None
        self.mutex.acquire()
        if self.record is None:
            if not self.check_record_availability(client):
                self.mutex.release()
                return TriggerResponse(success=False,
                                       message="Did not receive status or heartbeat, can not start recording.")
            bag_name = f'all_topics.bag'
            now = datetime.datetime.now()
            now = now.strftime("%Y%m%d_%H%M%S")
            dir_name = os.path.join(self.base, now)
            if not os.path.exists(dir_name):
                os.makedirs(dir_name)
            current_bag_full_path = os.path.join(dir_name, bag_name)
            rospy.set_param('/record/ctrl/save_directory', dir_name)
            meta_config = {"episode": int(self.datetime_start) * 1000 + self.i}
            self.record = Record(current_bag_full_path, client=client, meta=meta_config)
            self.record.start()
            rospy.loginfo(f"\n--- Start record {self.record.bag_path} ---")
            self.mutex.release()
            return TriggerResponse(success=True, message=current_bag_full_path)
        else:
            self.mutex.release()
            return TriggerResponse(success=False, message="Recording is already running.")

    def end_record_service(self, req):
        self.mutex.acquire()
        if self.record is not None:
            self.record.end()
            rospy.loginfo(f"\n--- End record {self.record.bag_path} ---")
            rospy.set_param('/record/ctrl/recording', False)
            save_path = self.record.full_path
            response = TriggerResponse(success=True, message=save_path)
            self.record = None
            self.mutex.release()
            convertor.add(save_path)
            return response
        else:
            self.mutex.release()
            return TriggerResponse(success=False, message="No recording is currently running.")

    def client_heartbeat_callback(self, msg):
        client = msg.data
        self.heartbeat_timestamps[client] = rospy.get_time()
        self.heartbeat_timestamps[None] = rospy.get_time()
        # rospy.loginfo(f"Heartbeat received from client {client}")

    def env_callback(self, msg):
        self.env_timestamps = rospy.get_time()


class Convertor:
    def __init__(self, base, fix_save_path=True, delete_bag=True):
        self.save_queue = queue.Queue()
        self.base = base
        if not os.path.exists(self.base):
            os.makedirs(self.base)
        self.path = None
        self.fix_save_path = fix_save_path
        self.delete_bag = delete_bag
        if fix_save_path:
            rospy.set_param('/record/ctrl/fixed_lmdb', True)
            self.path = self._get_path(name=None)
            rospy.loginfo(f"[record] LMDB save path is fixed, set to {self.path}.")
        else:
            rospy.set_param('/record/ctrl/fixed_lmdb', False)
            rospy.loginfo(f"[record] LMDB save path is not fixed.")

        convert_thread = threading.Thread(target=self._convert)
        convert_thread.start()

    def add(self, bag_name):
        self.save_queue.put(bag_name)

    def _get_path(self, name=None):
        datetime_start = datetime.datetime.now()
        if name is None:
            name = datetime_start.strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.base, name)

    def _convert(self):
        while not rospy.is_shutdown():
            if self.save_queue.empty():
                rospy.sleep(1)
                continue
            bag_full_path = self.save_queue.get()
            path = self.path if self.path is not None else self._get_path()
            rospy.loginfo(f"[record] get {bag_full_path}, start saving to {path}...")
            try:
                rospy.sleep(1)
                record(bag_full_path, lmdb_save_path=path)
                if self.delete_bag:
                    try:
                        os.remove(bag_full_path)
                    except OSError as e:
                        rospy.logerr(f"Converting Success, but can not delete the original rosbag file.")
            except Exception as e:
                traceback.print_exc()
                rospy.logerr(f"[record] Saving {bag_full_path} error: {e}...")


convertor = Convertor(lmdb_save_base, fix_save_path=lmdb_save_path_is_fixed)
recorder = rosbagRecorder(bag_save_base)
client_heartbeat_subscriber = rospy.Subscriber("/record/ctrl/client", String, recorder.client_heartbeat_callback)
env_heartbeat_subscriber = rospy.Subscriber("/record/ctrl/env", String, recorder.env_callback)
start_service = rospy.Service('/record/ctrl/start_record_srv', Trigger, recorder.start_record_service)
end_service = rospy.Service('/record/ctrl/end_record_srv', Trigger, recorder.end_record_service)
rospy.spin()
