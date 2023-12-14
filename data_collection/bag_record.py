#!/usr/bin/python3
import json
import os
import datetime
import subprocess
import time

import airsim
import rospy
import rosbag
from std_msgs.msg import Int8

now = datetime.datetime.now()
name = now.strftime("%Y%m%d_%H%M%S")

record_length = 30
max_record = 4
dir_name = f'/home/radiance/projects/Swarm-Formation/bags/{name}/'

if not os.path.exists(dir_name):
    os.makedirs(dir_name)

rospy.init_node('record_bags')
rospy.set_param('/record/dir_name', dir_name)
rospy.set_param('/record/start_record', False)
rospy.set_param('/record/max_record', max_record)
rospy.set_param('/record/max_record', record_length)
rospy.loginfo("[bag] Init record_bags node")

while not rospy.is_shutdown():
    if rospy.get_param('/record/start_record', False):
        params = rospy.get_param_names()
        d = {k: str(rospy.get_param(k)) for k in params}
        with open(os.path.join(dir_name, 'params.json'), 'w') as f:
            json.dump(d, f)
        trust = {"trust": []}
        with open(os.path.join(dir_name, 'trust.json'), 'w') as f:
            json.dump(trust, f)
        break

rospy.loginfo(f"[bag] Record Start...")

last_i = -1
publisher = rospy.Publisher('/section', Int8, queue_size=10)
start_time = time.time()
bag_name = os.path.join(dir_name, f'all.bag')
# command = "source /home/radiance/projects/Swarm-Formation/devel/setup.bash;"
command = "rosbag record -a -O {} --duration={}s".format(os.path.join(dir_name, bag_name), max_record * record_length)
process = subprocess.Popen(command, shell=True)

while not rospy.is_shutdown():
    if time.time() - start_time >= max_record * record_length:
        break
    i = int((time.time() - start_time) / record_length)
    if i > last_i:
        rospy.loginfo(f"\n--- Start record part {i}/{max_record} ---\n")
        last_i = i
    rospy.sleep(1)
    publisher.publish(i)

process.wait()
process.terminate()
rospy.loginfo(f"[bag] Record Finished...")
rospy.set_param('/record/start_record', False)

rospy.sleep(3)
if not rospy.is_shutdown():
    rospy.signal_shutdown("Recording finished")
    global_client = airsim.MultirotorClient()
    global_client.confirmConnection()
    global_client.reset()
    os.system("pkill -f ros")
