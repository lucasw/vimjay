#!/usr/bin/env python
# Lucas Walter
# Configure a CameraInfo from a dynamic reconfigure interface

import rospy

from sensor_msgs.sg import CameraInfo


class DrCameraInfo:
    def __init__(self):
        rospy.init_node('dr_camera_info')

if __name__ == '__main__':
    dr_camera_info = DrCameraInfo()
    rospy.spin()
