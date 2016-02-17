#!/usr/bin/env python
# Lucas Walter
# GNU GPLv3
# Load an image from disk and publish it repeatedly

import cv2
import numpy as np
import rospy
import sys
from sensor_msgs.msg import Image

if __name__ == '__main__': 
    rospy.init_node('image_publish')
    
    name = sys.argv[1]
    image = cv2.imread(name)
    #cv2.imshow("im", image)
    #cv2.waitKey(5)
    
    hz = rospy.get_param("~rate", 1)
    frame_id = rospy.get_param("~frame_id", "map")
    rate = rospy.Rate(hz)

    pub = rospy.Publisher('image', Image, queue_size=1)
    
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.encoding = 'bgr8'
    msg.height = image.shape[0]
    msg.width = image.shape[1]
    msg.step = image.shape[1] * 3
    msg.data = image.tostring()
    pub.publish(msg)

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()
