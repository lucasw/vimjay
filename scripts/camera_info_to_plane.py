#!/usr/bin/env python
# Lucas Walter
# Project the border of an image onto a plane using a camera info and tf

import copy

import cv2
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import (
    PolygonStamped,
    TransformStamped,
)
from sensor_msgs.msg import (
    CameraInfo,
)
from tf import transformations
from tf2_msgs.msg import TFMessage
from vimjay import (
    camera_info_to_cv2,
    points_in_camera_transform_to_plane,
    get_camera_edge_points,
    points_to_marker,
    points_to_polygon,
    transform_points,
)
from visualization_msgs.msg import (
    MarkerArray,
)


class CameraInfoToPlane:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.do_plane_pubs = rospy.get_param("~do_plane_pubs", True)

        self.marker_id = rospy.get_param("~marker_id", 0)
        # the frame to intersect the camera info with
        self.target_frame = rospy.get_param("~target_frame", "odom")
        # the frame to convert the intersection points into
        self.output_frame = rospy.get_param("~output_frame", "odom")
        self.camera_frame_override = rospy.get_param("~camera_frame_override", "")
        self.marker_pub = rospy.Publisher("marker_array", MarkerArray, queue_size=3)
        self.polygon_pub = rospy.Publisher("footprint", PolygonStamped, queue_size=3)

        self.plane_camera_frame = rospy.get_param("~plane_camera_frame", "")
        self.plane_camera_z = rospy.get_param("~plane_camera_z", 4.0)
        self.plane_camera_info_pub = rospy.Publisher("plane/camera_info", CameraInfo, queue_size=2)
        self.tf_pub = rospy.Publisher("/tf", TFMessage, queue_size=5)

        self.last_update = rospy.Time.now()
        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.camera_info_callback, queue_size=20)

    def camera_info_callback(self, camera_info: CameraInfo):
        # https://github.com/ros/geometry2/issues/43#issuecomment-487223454
        last_update = self.last_update
        cur_update = rospy.Time.now()
        self.last_update = cur_update
        if cur_update < last_update:
            rospy.logwarn("looping time, resetting tf buffer")
            self.tf_buffer.clear()
            return

        if self.camera_frame_override not in ["", None]:
            rospy.logwarn_once(f"overriding '{camera_info.header.frame_id}' with {self.camera_frame_override}")
            camera_info.header.frame_id = self.camera_frame_override

        try:
            tfs0 = self.tf_buffer.lookup_transform(self.target_frame, camera_info.header.frame_id,
                                                   camera_info.header.stamp, rospy.Duration(0.3))
            camera_frame_to_target_plane_tfs = tfs0
            tfs1 = self.tf_buffer.lookup_transform(self.output_frame, self.target_frame,
                                                   camera_info.header.stamp, rospy.Duration(0.3))
            target_to_output_tfs = tfs1
        except (tf2_ros.ConnectivityException, tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                rospy.exceptions.ROSTimeMovedBackwardsException) as ex:
            rospy.logwarn_throttle(4.0, ex)
            return

        camera_matrix, dist_coeff, rvec, tvec = camera_info_to_cv2(camera_info)

        edge_points = get_camera_edge_points(camera_info, num_per_edge=8)

        rv = points_in_camera_transform_to_plane(camera_frame_to_target_plane_tfs,
                                                 edge_points, camera_matrix, dist_coeff)
        points_in_plane, _, is_full = rv

        marker_in_camera_frame = True
        if marker_in_camera_frame:
            try:
                tfs_inv = self.tf_buffer.lookup_transform(camera_info.header.frame_id, self.target_frame,
                                                          camera_frame_to_target_plane_tfs.header.stamp,
                                                          rospy.Duration(0.0))
            except (tf2_ros.ConnectivityException, tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    rospy.exceptions.ROSTimeMovedBackwardsException) as ex:
                rospy.logwarn_throttle(4.0, ex)
                return

            points_in_plane_in_camera = transform_points(points_in_plane, tfs_inv)

            marker = points_to_marker(camera_info.header.stamp, camera_info.header.frame_id,
                                      points_in_plane_in_camera, marker_id=self.marker_id)
        else:
            marker = points_to_marker(camera_info.header.stamp, self.target_frame,
                                      points_in_plane, marker_id=self.marker_id)

        marker.color.r -= self.marker_id / 5.0
        marker.color.b += self.marker_id / 8.0

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

        if not (is_full and self.do_plane_pubs):
            return

        # just get corners
        corner_points = get_camera_edge_points(camera_info, num_per_edge=1)
        points, points2d, _ = points_in_camera_transform_to_plane(camera_frame_to_target_plane_tfs, corner_points,
                                                                  camera_matrix, dist_coeff)

        points_in_output = transform_points(points, target_to_output_tfs)
        polygon = points_to_polygon(camera_info.header.stamp, self.output_frame, points_in_output)
        self.polygon_pub.publish(polygon)

        # corners of image in pixel coordinates
        input_pts = np.float32([[points2d[0][0], points2d[0][1]],
                                [points2d[1][0], points2d[1][1]],
                                [points2d[2][0], points2d[2][1]],
                                [points2d[3][0], points2d[3][1]],
                                ])

        output_pts = np.float32([[points[0].x, points[0].y],
                                 [points[1].x, points[1].y],
                                 [points[2].x, points[2].y],
                                 [points[3].x, points[3].y],
                                 ])

        plane_x_min = np.min(output_pts[:, 0])
        plane_x_max = np.max(output_pts[:, 0])
        plane_y_min = np.min(output_pts[:, 1])
        plane_y_max = np.max(output_pts[:, 1])

        plane_x_range = plane_x_max - plane_x_min
        plane_y_range = plane_y_max - plane_y_min

        # TODO(lucasw) dynamic reconfigure
        scale = 800.0 / plane_x_range
        output_pts[:, 0] -= plane_x_min
        output_pts[:, 0] *= scale

        output_pts[:, 1] -= plane_y_min
        output_pts[:, 1] *= scale

        rospy.logdebug_throttle(2.0, f"\n{input_pts} ->\n{output_pts}")
        perspective_transform = cv2.getPerspectiveTransform(input_pts, output_pts)
        rospy.logdebug_throttle(2.0, f"{perspective_transform}")

        wd = np.max(output_pts[:, 0])
        ht = np.max(output_pts[:, 1])

        # publish a transform of where the virtual plane camera is looking from
        tfm = TFMessage()
        tfs = TransformStamped()
        tfs.header.stamp = camera_info.header.stamp

        plane_camera_frame = self.plane_camera_frame
        if plane_camera_frame == "":
            plane_camera_frame = self.target_frame + "_" + camera_info.header.frame_id

        tfs0 = copy.deepcopy(tfs)
        tfs0.header.frame_id = self.target_frame
        tfs0.child_frame_id = plane_camera_frame + "_xy"
        tfs0.transform.translation.x = plane_x_min + plane_x_range * 0.5
        tfs0.transform.translation.y = plane_y_min + plane_y_range * 0.5
        tfs0.transform.rotation.w = 1.0
        tfm.transforms.append(tfs0)

        tfs.header.frame_id = plane_camera_frame + "_xy"
        tfs.child_frame_id = plane_camera_frame
        tfs.transform.translation.z = self.plane_camera_z
        quat = transformations.quaternion_from_euler(0.0, np.pi, np.pi * 0.5)
        tfs.transform.rotation.x = quat[0]
        tfs.transform.rotation.y = quat[1]
        tfs.transform.rotation.z = quat[2]
        tfs.transform.rotation.w = quat[3]
        tfm.transforms.append(tfs)

        self.tf_pub.publish(tfm)

        ci = CameraInfo()
        ci.header.stamp = camera_info.header.stamp
        ci.header.frame_id = plane_camera_frame
        ci.width = int(wd)
        ci.height = int(ht)
        ci.distortion_model = "plumb_bob"

        # TODO(lucasw) need a way to set these values- have this node
        # subscribe to an input CameraInfo?
        cx = wd / 2.0
        cy = ht / 2.0
        fx = cx * self.plane_camera_z / (plane_y_range * 0.5)
        fy = cy * self.plane_camera_z / (plane_x_range * 0.5)
        ci.D = [0.0, 0.0, 0.0, 0, 0]
        ci.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        ci.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.plane_camera_info_pub.publish(ci)


if __name__ == "__main__":
    rospy.init_node("camera_info_to_plane")
    node = CameraInfoToPlane()
    rospy.spin()
