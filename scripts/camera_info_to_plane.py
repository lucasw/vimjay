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
from visualization_msgs.msg import (
    Marker,
    MarkerArray,
)


# TODO(lucasw) move into src and make an importable function of it
# from https://github.com/lucasw/sdl2_ros sdl2_ros/scripts/sprites.py
def camera_info_to_cv2(camera_info: CameraInfo) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    rvec = np.zeros((1, 3))
    tvec = np.zeros((1, 3))
    camera_matrix = np.zeros((3, 3))
    for y_ind in range(3):
        for x_ind in range(3):
            ind = y_ind * 3 + x_ind
            camera_matrix[y_ind, x_ind] = camera_info.K[ind]
    # cx = camera_info.K[1]
    # cy = camera_info.K[5]
    dist_coeff = np.asarray(camera_info.D)
    return camera_matrix, dist_coeff, rvec, tvec


class CameraInfoToPlane:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_id = rospy.get_param("~marker_id", 0)
        self.target_frame = rospy.get_param("~target_frame", "odom")
        self.marker_pub = rospy.Publisher("marker_array", MarkerArray, queue_size=3)
        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.camera_info_callback, queue_size=20)

    def camera_info_callback(self, msg: CameraInfo):
        try:
            tfs = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id,
                                                  msg.header.stamp, rospy.Duration(0.3))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn_throttle(4.0, ex)
            return

        camera_matrix, dist_coeff, rvec, tvec = camera_info_to_cv2(msg)

        num_per_edge = 8
        num_points = num_per_edge * 4 + 1

        points2d = np.zeros((num_points, 2))
        ind = 0
        # create points in a loop around the edge of the image
        for i in range(num_per_edge):
            fr = i / num_per_edge
            points2d[ind, 0] = fr * msg.width
            ind += 1

        for i in range(num_per_edge):
            fr = i / num_per_edge
            points2d[ind, 0] = msg.width
            points2d[ind, 1] = fr * msg.height
            ind += 1

        for i in range(num_per_edge):
            fr = 1.0 - i / num_per_edge
            points2d[ind, 0] = fr * msg.width
            points2d[ind, 1] = msg.height
            ind += 1

        # complete the loop with the + 1
        for i in range(num_per_edge + 1):
            fr = 1.0 - i / num_per_edge
            points2d[ind, 1] = fr * msg.height
            ind += 1

        # idealized points all at range 1.0
        ideal_points = cv2.undistortPoints(points2d, camera_matrix, dist_coeff)

        points3d_in_camera = np.ones((num_points + 1, 3))
        points3d_in_camera[:-1, 0] = ideal_points[:, 0, 0]
        points3d_in_camera[:-1, 1] = ideal_points[:, 0, 1]

        # the origin of the camera
        points3d_in_camera[-1, 0] = 0.0
        points3d_in_camera[-1, 1] = 0.0
        points3d_in_camera[-1, 2] = 0.0

        # rospy.loginfo_throttle(6.0, f"\n{points2d}")
        # rospy.loginfo_throttle(6.0, f"\n{points3d_in_camera}")

        # put numpy/cv2 points into a point cloud
        field_points = []
        for ind in range(points3d_in_camera.shape[0]):
            x = points3d_in_camera[ind, 0]
            y = points3d_in_camera[ind, 1]
            z = points3d_in_camera[ind, 2]
            pt = [x, y, z]
            field_points.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  ]

        pc2_in = point_cloud2.create_cloud(msg.header, fields, field_points)
        # pc2_in.header.stamp = msg.header.stamp
        # TODO(lucasw) probably a more efficient transform on a Point array than converting to PointCloud2
        pc2_out = do_transform_cloud(pc2_in, tfs)
        pc2_points = point_cloud2.read_points(pc2_out, field_names=("x", "y", "z"), skip_nans=True)

        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = self.target_frame
        marker.ns = "camera_info"
        marker.id = self.marker_id
        marker.type = Marker.LINE_STRIP
        marker.pose.orientation.w = 1.0
        marker.lifetime = rospy.Duration(0.0)
        marker.frame_locked = False
        marker.scale.x = 0.05
        marker.color.r = 0.7
        marker.color.g = 1.0
        marker.color.a = 1.0

        # convert from generator to list
        pc2_points = [pt for pt in pc2_points]

        # the camera origin in the target frame
        pt0 = pc2_points[-1]
        x0 = pt0[0]
        y0 = pt0[1]
        z0 = pt0[2]

        for pt1 in pc2_points[:-1]:
            x1 = pt1[0]
            y1 = pt1[1]
            z1 = pt1[2]
            # is the ray facing away from the plane, or parallel, and will never intersect?
            if z1 >= z0 and z0 >= 0.0:
                continue
            elif z1 >= z0 and z0 <= 0.0:
                continue

            scale = z0 / (z0 - z1)
            x2 = x0 + (x1 - x0) * scale
            y2 = y0 + (y1 - y0) * scale
            # this should be 0.0
            z2 = z0 + (z1 - z0) * scale

            pt = Point(x2, y2, z2)
            # pt = Point(x1, y1, z1)
            marker.points.append(pt)

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    rospy.init_node("camera_info_to_plane")
    node = CameraInfoToPlane()
    rospy.spin()
