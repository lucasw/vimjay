#!/usr/bin/env python
# Lucas Walter
# Configure a CameraInfo from a dynamic reconfigure interface

import rospy

from dynamic_reconfigure.server import Server
from sensor_msgs.msg import CameraInfo
from vimjay.cfg import DrCameraInfoConfig


class DrCameraInfo:
    def __init__(self):
        rospy.init_node('dr_camera_info')
        self.camera_info = None
        self.pub = rospy.Publisher("camera_info", CameraInfo, queue_size=1, latch=True)
        self.server = Server(DrCameraInfoConfig, self.dr_callback)
        # reset=True makes this node survive jumps back in time (why not make that the default?)
        # https://github.com/ros-visualization/interactive_markers/pull/47/
        # TODO(lucasw) make this update if the callback changes update rate
        self.timer = rospy.Timer(rospy.Duration(0.2), self.update, reset=True)

    def dr_callback(self, config, level):
        ci = CameraInfo()
        ci.header.frame_id = config['frame_id']
        ci.width = config['width']
        ci.height = config['height']
        ci.distortion_model = config['distortion_model']
        ci.D = [config['d0'], config['d1'], config['d2'], config['d3'], config['d4']]
        ci.K[0 * 3 + 0] = config['fx']
        ci.K[0 * 3 + 2] = config['cx']
        ci.K[1 * 3 + 1] = config['fy']
        ci.K[1 * 3 + 2] = config['cy']
        ci.K[2 * 3 + 2] = 1
        ci.P[0 * 4 + 0] = config['fx']
        ci.P[0 * 4 + 2] = config['cx']
        ci.P[1 * 4 + 1] = config['fy']
        ci.P[1 * 4 + 2] = config['cy']
        ci.P[2 * 4 + 2] = 1
        ci.R[0] = 1
        ci.R[4] = 1
        ci.R[8] = 1
        self.camera_info = ci
        return config

    def update(self, event):
        self.camera_info.header.stamp = rospy.Time.now()
        self.pub.publish(self.camera_info)


if __name__ == '__main__':
    dr_camera_info = DrCameraInfo()
    rospy.spin()
