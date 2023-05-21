#!/usr/bin/env python
# Lucas Walter
# GNU GPLv3
# Load an image from disk and publish a resized version of it repeatedly as triggered by incoming camera info

import cv2
import rospy
import sys
from sensor_msgs.msg import CameraInfo, Image


class ImagePub:
    def __init__(self, name):
        self.image = cv2.imread(name, cv2.IMREAD_COLOR)
        rospy.loginfo(f"{name} {self.image.shape}")

        self.image_pub = rospy.Publisher("image", Image, queue_size=1)

        self.ci_sub = rospy.Subscriber("camera_info_in", CameraInfo,
                                       self.camera_info_callback, queue_size=1)

    def camera_info_callback(self, camera_info: CameraInfo):
        # TODO(lucasw) or use cv bridge or ros_numpy
        image = Image()
        image.header = camera_info.header
        image.encoding = "bgr8"
        image = cv2.resize(self.image, (camera_info.width, camera_info.height))
        image.height = image.shape[0]
        image.width = image.shape[1]
        image.step = image.shape[1] * 3
        image.data = image.tobytes()
        self.image_pub.publish(image)


if __name__ == "__main__":
    rospy.init_node("image_publish")
    name = sys.argv[1]
    node = ImagePub(name)
    rospy.spin()
