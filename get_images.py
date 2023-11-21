#!/usr/bin/env python
import rospy
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisher:
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.camera_number = rospy.get_param('~camera_number', 0)
        # self.ip = rospy.get_param('~ip', 'localhost')
        # self.port = rospy.get_param('~port', '8080')
        self.camera_type = rospy.get_param('~camera_type', 'usb')
        self.bridge = CvBridge()
        self.camera_name = None
        if self.camera_type == "usb":
            self.cap_usb = cv2.VideoCapture(self.camera_number)
            if not self.cap_usb.isOpened():
                rospy.logwarn(f"Error opening camera with id {self.camera_number}, exiting this camera node.")
                exit()
            rospy.logwarn(f"Launch camera node with id {self.camera_number}...")

            self.camera_name = f"camera_{self.camera_type}_{self.camera_number}"
            self.pub = rospy.Publisher(f'/{self.camera_name}/image_raw', Image,
                                       queue_size=10)

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
