#!/usr/bin/env python
# Lucas Walter
# GNU GPLv3
# Load an image from disk and publish it repeatedly

import cv2
import numpy as np
import rospy
import sys
from sensor_msgs.msg import CameraInfo, Image

if __name__ == '__main__': 
    rospy.init_node('image_publish')
    
    name = sys.argv[1]
    image = cv2.imread(name)
    #cv2.imshow("im", image)
    #cv2.waitKey(5)
    
    hz = rospy.get_param("~rate", 1)
    frame_id = rospy.get_param("~frame_id", "map")
    use_image = rospy.get_param("~use_image", True)
    rate = rospy.Rate(hz)

    if use_image:
        pub = rospy.Publisher('image', Image, queue_size=1)
    ci_pub = rospy.Publisher('camera_info', CameraInfo, queue_size=1)
    
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.encoding = 'bgr8'
    msg.height = image.shape[0]
    msg.width = image.shape[1]
    msg.step = image.shape[1] * 3
    msg.data = image.tostring()
    if use_image:
        pub.publish(msg)

    ci = CameraInfo()
    ci.header = msg.header
    ci.height = msg.height
    ci.width = msg.width
    ci.distortion_model ="plumb_bob"
    # TODO(lucasw) need a way to set these values- have this node
    # subscribe to an input CameraInfo?
    ci.D = [0.0, 0.0, 0.0, 0, 0]
    ci.K = [500.0, 0.0, msg.width/2, 0.0, 500.0, msg.height/2, 0.0, 0.0, 1.0]
    ci.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.P = [500.0, 0.0, msg.width/2, 0.0, 0.0, 500.0, msg.height/2, 0.0,  0.0, 0.0, 1.0, 0.0]
    ci_pub.publish(ci)

    while not rospy.is_shutdown():
        # the ci header is the same object so also gets updated
        msg.header.stamp = rospy.Time.now()
        if use_image:
            pub.publish(msg)
        ci_pub.publish(ci)
        rate.sleep()
