#!/usr/bin/python3
import rospy
from std_srvs.srv import Trigger


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
    rospy.wait_for_service('/record/ctrl/end_record_srv')
    try:
        end_record = rospy.ServiceProxy('/record/ctrl/end_record_srv', Trigger)
        response = end_record()
        if response.success:
            rospy.loginfo("Recording stopped successfully.")
        else:
            rospy.loginfo("Unable to stop recording: " + response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: " + str(e))


def set_ros_param():
    rospy.loginfo("Enter value to set for /env/info/instruct: ")
    param_value = input(">>> ")
    if param_value.endswith("3"):
        param_value = param_value.rstrip("3")
    rospy.set_param('/env/info/instruct', param_value)
    final_param_value = rospy.get_param('/env/info/instruct', "")
    rospy.loginfo(f"Parameter /env/info/instruct set to: {final_param_value}")


def main():
    rospy.init_node('record_control_client')
    rospy.loginfo("Record Control Client Started")
    tutorial = """
Press 's' to start recording, 'e' to end recording, 'p' to set param for \"/env/info/instruct\", and 'q' to quit.
"""
    rospy.loginfo(tutorial)

    while not rospy.is_shutdown():
        command = input(">>> ")
        if command == 's' or command == '1':
            send_start_request()
        elif command == 'e' or command == '2':
            send_end_request()
        elif command == 'p' or command == '3':
            set_ros_param()
        elif command == 'q':
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
