#!/usr/bin/python3
import json
import os
import shutil
import time

import rospy
from std_srvs.srv import Trigger

last_end = time.time() - 10
last_press = time.time()
wait_time_record = 1
wait_time_press = 0.5
records_bag_full_path = []


def send_start_request():
    rospy.wait_for_service('/record/ctrl/start_record_srv')
    try:
        # Check if '/env/info/instruct' is set and not empty
        if not rospy.has_param('/env/info/instruct') or rospy.get_param('/env/info/instruct') == "":
            rospy.logwarn("Warning: Parameter '/env/info/instruct' is not set or is empty.")

        start_record = rospy.ServiceProxy('/record/ctrl/start_record_srv', Trigger)
        response = start_record()
        if response.success:
            instruct = rospy.get_param('/env/info/instruct', "")
            rospy.loginfo(f"Recording started successfully, current instruction is \"{instruct}\".")
        else:
            rospy.loginfo("Unable to start recording: " + response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: " + str(e))


def send_end_request():
    global last_end
    rospy.wait_for_service('/record/ctrl/end_record_srv')
    try:
        end_record = rospy.ServiceProxy('/record/ctrl/end_record_srv', Trigger)
        response = end_record()
        if response.success:
            bag_full_path = response.message
            records_bag_full_path.append(bag_full_path)
            rospy.loginfo(f"Recording stopped successfully, save to {bag_full_path}.")
        else:
            rospy.loginfo("Unable to stop recording: " + response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: " + str(e))
    last_end = time.time()


def set_ros_param():
    rospy.loginfo("Enter value to set for /env/info/instruct: ")
    param_value = input(">>> ")
    if param_value.endswith("3"):
        param_value = param_value.rstrip("3")
    rospy.set_param('/env/info/instruct', param_value)
    final_param_value = rospy.get_param('/env/info/instruct', "")
    rospy.loginfo(f"Parameter /env/info/instruct set to: {final_param_value}")


def list_bag():
    if not records_bag_full_path:
        rospy.loginfo("No ROS bags available.")
        return

    bags_info = "Available ROS bags (only this life cycle is supported now):\n"
    bags_info += "\n".join(f"{i}: {bag_path}" for i, bag_path in enumerate(records_bag_full_path))

    rospy.loginfo(bags_info)


def delete_bag():
    if not records_bag_full_path:
        rospy.logwarn("No ROS bags available for deletion.")
        return
    fixed_lmdb = rospy.get_param('/record/ctrl/fixed_lmdb', False)
    if fixed_lmdb:
        rospy.logwarn(f"The value of '/record/ctrl/fixed_lmdb' is set to True.")
        rospy.logwarn(f"Finding itmes in the unified lmdb dataset is not supported now.")
        rospy.logwarn(f"The delete behavior will only delete rosbag file and not affect items in lmdb.")

    while not rospy.is_shutdown():

        while not rospy.is_shutdown():
            if not records_bag_full_path:
                rospy.loginfo("Records is empty now, returning to main process...\n")
                return
            list_bag()
            rospy.loginfo("Enter 'q' to exit; Enter the index of the ROS bag to delete:")
            try:
                num = input(">>> ")
                if num == "q":
                    return
                index = int(num)
                if index < 0 or index >= len(records_bag_full_path):
                    raise ValueError
                break
            except ValueError:
                rospy.logerr("Invalid index entered. Please enter a valid numerical index.")

        bag_full_path = records_bag_full_path[index]
        bag_directory = os.path.dirname(bag_full_path)
        json_full_name = os.path.join(bag_directory, 'meta.json')

        try:
            with open(json_full_name, 'r') as file:
                meta_data = json.load(file)
            save_path = meta_data.get("save_path")

            if fixed_lmdb and save_path and os.path.exists(save_path):
                rospy.logerr(
                    f"Deletion of save path '{save_path}' is not permitted as '/record/ctrl/fixed_lmdb' is set to True.")
            elif save_path and os.path.exists(save_path):
                shutil.rmtree(save_path)
                rospy.loginfo(f"Associated save path '{save_path}' has been successfully deleted.")
        except Exception as e:
            rospy.logwarn(f"Error encountered while handling save path: {e}")

        try:
            shutil.rmtree(bag_directory)
            records_bag_full_path.pop(index)
            rospy.loginfo(f"ROS bag directory '{bag_directory}' has been successfully deleted.\n")
        except Exception as e:
            rospy.logerr(f"Error encountered while deleting ROS bag directory: {e}")


def main():
    global last_end, last_press
    rospy.init_node('record_control_client')
    rospy.loginfo("Record Control Client Started")
    tutorial = """
Press 'q' to quit, 
Press 'r' to start or end recording (Pedal 1, intelligent), 
Press 's' to start recording, 'e' to end recording, 
Press 'p' to set param for \"/env/info/instruct\",
Input 'ls' to list the records (Pedal 3) (only this life cycle is supported now),
Input 'del' to enter into the delete program.
"""
    rospy.loginfo(tutorial)

    while not rospy.is_shutdown():
        command = input(">>> ")
        if command == '':
            continue

        if not time.time() - last_press > wait_time_press:
            rospy.logwarn("Operation ignored: Please wait a moment before pressing again.")
            time.sleep(0.2)
            continue
        last_press = time.time()

        if not time.time() - last_end > wait_time_record:
            rospy.logwarn("Operation ignored: Please wait a moment before recording again.")
            time.sleep(0.5)
            continue

        if command == 'r' or command == '1':
            if rospy.get_param('/record/ctrl/recording', False):
                # if it is recording
                command = 'e'
            else:
                command = 's'
        elif command == "3":
            command = 'ls'

        if command == 's':
            send_start_request()
        elif command == 'e':
            send_end_request()
        elif command == 'p':
            set_ros_param()
        elif command == 'ls':
            list_bag()
        elif command == 'del':
            rospy.loginfo(f"Getting into deleting program...\n")
            delete_bag()
        elif 'q' in command:
            if rospy.get_param('/record/ctrl/recording', False):
                rospy.logerr("Cannot exit while recording is in progress.")
            else:
                rospy.loginfo("Exiting.")
                break
        else:
            rospy.logerr("Invalid command.")
            rospy.loginfo(tutorial)


if __name__ == '__main__':
    main()
