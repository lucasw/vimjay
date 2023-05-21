#!/usr/bin/env python
# Lucas Walter
# Project the border of an image onto a plane using a camera info and tf

import cv2
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from sensor_msgs.msg import (
    CameraInfo,
    PointField,
)
from sensor_msgs import point_cloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from vimjay import (
    camera_info_to_plane,
    points_to_marker,
)
from visualization_msgs.msg import (
    Marker,
    MarkerArray,
)


class CameraInfoToPlane:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_id = rospy.get_param("~marker_id", 0)
        self.target_frame = rospy.get_param("~target_frame", "odom")
        self.marker_pub = rospy.Publisher("marker_array", MarkerArray, queue_size=3)
        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.camera_info_callback, queue_size=20)

    def camera_info_callback(self, msg: CameraInfo):
        points = camera_info_to_plane(self.tf_buffer, msg, self.target_frame)
        marker = points_to_marker(msg.header.stamp, self.target_frame, points, marker_id=self.marker_id)

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node("camera_info_to_plane")
    node = CameraInfoToPlane()
    rospy.spin()
