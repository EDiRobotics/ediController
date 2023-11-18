#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisher:
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.camera_number = rospy.get_param('~camera_number', 0)
        self.ip = rospy.get_param('~ip', 'localhost')
        self.port = rospy.get_param('~port', '8080')
        self.camera_type = rospy.get_param('~camera_type', 'usb')
        self.bridge = CvBridge()
        if self.camera_type == "usb":
            self.pub = rospy.Publisher(f'/camera_{self.camera_type}_{self.camera_number}/image_raw', Image, queue_size=10)
            self.cap_usb = cv2.VideoCapture(self.camera_number)

    def start(self):
        if self.camera_type == "usb":
            self.publish_usb_image()

    def publish_usb_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap_usb.read()
            if ret:
                try:
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.pub.publish(ros_image)
                except Exception as e:
                    print(e)


if __name__ == '__main__':
    image_publisher = ImagePublisher()
    image_publisher.start()
