#!/usr/bin/env python
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

"""
TODO
"""


class ImagePublisher:
    def __init__(self):
        rospy.init_node('tof_publisher', anonymous=True)
        self.camera_number = rospy.get_param('~camera_number', 0)
        # self.ip = rospy.get_param('~ip', 'localhost')
        # self.port = rospy.get_param('~port', '8080')
        self.camera_type = rospy.get_param('~camera_type', 'usb')
        self.bridge = CvBridge()
        self.camera_name = None
        if self.camera_type == "usb":
            self.cap_usb = cv2.VideoCapture(self.camera_number, cv2.CAP_V4L2)
            if not self.cap_usb.isOpened():
                rospy.logwarn(f"Error opening camera with id {self.camera_number}, exiting this camera node.")
                exit()
            frame_width = self.cap_usb.get(cv2.CAP_PROP_FRAME_WIDTH)
            frame_height = self.cap_usb.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.camera_name = f"tof_{self.camera_type}_{self.camera_number}"
            topic = f'/{self.camera_name}/image_raw'
            self.pub = rospy.Publisher(topic, Image,
                                       queue_size=1)
            rospy.loginfo(f"Launch camera {self.camera_number} on {topic}, frame size: {frame_width}x{frame_height}")

    def start(self):
        if self.camera_type == "usb":
            self.publish_usb_image()

    def publish_usb_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap_usb.read()
            if ret:
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now()
                header.frame_id = self.camera_name
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8", header=header)
                    self.pub.publish(ros_image)
                except Exception as e:
                    print(e)


if __name__ == '__main__':
    image_publisher = ImagePublisher()
    image_publisher.start()
