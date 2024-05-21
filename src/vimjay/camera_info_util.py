# Lucas Walter
# Project the border of an image onto a plane using a camera info and tf

from __future__ import annotations

import copy

import cv2
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import (
    Point,
    Point32,
    PolygonStamped,
    TransformStamped,
)
from sensor_msgs.msg import (
    CameraInfo,
    PointCloud2,
    PointField,
)
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker


# TODO(lucasw) move into src and make an importable function of it
# from https://github.com/lucasw/sdl2_ros sdl2_ros/scripts/sprites.py
def camera_info_to_cv2(camera_info: CameraInfo) -> (np.ndarray, np.ndarray, np.ndarray, np.ndarray):
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


def get_camera_edge_points(camera_info: CameraInfo, num_per_edge=8) -> np.ndarray:
    num_points = num_per_edge * 4 + 1

    points2d = np.zeros((num_points, 2))
    ind = 0
    # create points in a loop around the edge of the image
    for i in range(num_per_edge):
        fr = i / num_per_edge
        points2d[ind, 0] = fr * camera_info.width
        ind += 1

    for i in range(num_per_edge):
        fr = i / num_per_edge
        points2d[ind, 0] = camera_info.width
        points2d[ind, 1] = fr * camera_info.height
        ind += 1

    for i in range(num_per_edge):
        fr = 1.0 - i / num_per_edge
        points2d[ind, 0] = fr * camera_info.width
        points2d[ind, 1] = camera_info.height
        ind += 1

    # complete the loop with the + 1
    for i in range(num_per_edge + 1):
        fr = 1.0 - i / num_per_edge
        points2d[ind, 1] = fr * camera_info.height
        ind += 1

    return points2d


def points3d_np_to_pointcloud2(points3d: np.ndarray, header: Header) -> PointCloud2:
    field_points = []
    for ind in range(points3d.shape[0]):
        x = points3d[ind, 0]
        y = points3d[ind, 1]
        z = points3d[ind, 2]
        pt = [x, y, z]
        field_points.append(pt)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              ]

    pc2 = point_cloud2.create_cloud(header, fields, field_points)
    return pc2


def points3d_to_pointcloud2(points3d: [Point], header: Header) -> PointCloud2:
    field_points = []
    for pt in points3d:
        pt = [pt.x, pt.y, pt.z]
        field_points.append(pt)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              ]

    pc2 = point_cloud2.create_cloud(header, fields, field_points)
    return pc2


# TODO(lucasw) is there a an off-the-shelf function that does this?
def transform_points_pc2(points3d: np.ndarray, transform: TransformStamped) -> PointCloud2:
    """
    points3d is n x 3 in the src_header.frame_id frame
    transform child frame id should match src_header frame id
    transform header frame id is the destination frame
    """
    src_header = Header()
    src_header.frame_id = transform.child_frame_id
    src_header.stamp = transform.header.stamp
    pc2_in = points3d_to_pointcloud2(points3d, src_header)

    # pc2_in.header.stamp = camera_info.header.stamp
    # TODO(lucasw) probably a more efficient transform on a Point array than converting to PointCloud2
    pc2_out = do_transform_cloud(pc2_in, transform)
    return pc2_out


def transform_points_np_pc2(points3d: np.ndarray, transform: TransformStamped) -> PointCloud2:
    """
    points3d is n x 3 in the src_header.frame_id frame
    transform child frame id should match src_header frame id
    transform header frame id is the destination frame
    """
    src_header = Header()
    src_header.frame_id = transform.child_frame_id
    src_header.stamp = transform.header.stamp
    pc2_in = points3d_np_to_pointcloud2(points3d, src_header)

    # pc2_in.header.stamp = camera_info.header.stamp
    # TODO(lucasw) probably a more efficient transform on a Point array than converting to PointCloud2
    pc2_out = do_transform_cloud(pc2_in, transform)
    return pc2_out


def transform_points_np(points3d: np.ndarray, transform: TransformStamped) -> np.ndarray:
    pc2_out = transform_points_np_pc2(points3d, transform)
    pc2_points = point_cloud2.read_points(pc2_out, field_names=("x", "y", "z"), skip_nans=True)
    # convert from generator to list
    pc2_points = [pt for pt in pc2_points]
    transformed_points3d = np.zeros((len(pc2_points), 3))

    for ind, pt in enumerate(pc2_points):
        transformed_points3d[ind, :] = pt

    return transformed_points3d


def transform_points(points_in_src: list[Point], tfs: TransformStamped) -> list[Point]:
    points_in_src_np = points_list_to_array(points_in_src)
    points_in_dst_np = transform_points_np(points_in_src_np, tfs)
    points_in_dst = points_array_to_list(points_in_dst_np)
    return points_in_dst


def points_list_to_array(points: [Point]) -> np.ndarray:
    points_np = np.zeros((len(points), 3))
    for ind, pt in enumerate(points):
        points_np[ind, 0] = pt.x
        points_np[ind, 1] = pt.y
        points_np[ind, 2] = pt.z
    return points_np


def points_array_to_list(points_np: np.ndarray) -> [Point]:
    points = []
    for ind in range(points_np.shape[0]):
        points.append(Point(points_np[ind, 0], points_np[ind, 1], points_np[ind, 2]))
    return points


# TODO(lucasw) it doesn't add a whole lot of value to have this be separate from camera_points
def points_in_camera_transform_to_plane(camera_to_target_transform: TransformStamped,
                                        points2d_in_camera: np.ndarray,
                                        camera_matrix: np.ndarray,
                                        dist_coeff: np.ndarray) -> ([Point], [np.ndarray], bool):
    # idealized points all at range 1.0
    ideal_points = cv2.undistortPoints(points2d_in_camera, camera_matrix, dist_coeff)

    num_points = points2d_in_camera.shape[0]
    points3d_in_camera = np.ones((num_points + 1, 3))

    points3d_in_camera[:-1, 0] = ideal_points[:, 0, 0]
    points3d_in_camera[:-1, 1] = ideal_points[:, 0, 1]

    # the origin of the camera
    points3d_in_camera[-1, 0] = 0.0
    points3d_in_camera[-1, 1] = 0.0
    points3d_in_camera[-1, 2] = 0.0

    # rospy.loginfo_throttle(6.0, f"\n{points2d_in_camera}")
    # rospy.loginfo_throttle(6.0, f"\n{points3d_in_camera}")

    return points3d_to_plane(camera_to_target_transform,
                             points3d_in_camera,
                             points2d_in_camera)


def points3d_to_plane_np(camera_to_target_transform: TransformStamped,
                         points3d_in_camera: np.ndarray) -> (np.ndarray, np.ndarray):
    """
    The final point in the input 3d array needs to be the sensor origin
    """
    pc2_points = transform_points_np(points3d_in_camera, camera_to_target_transform)

    # t0 = rospy.Time.now()
    # the camera origin in the target frame
    pt0 = pc2_points[-1, :]
    x0 = pt0[0]
    y0 = pt0[1]
    z0 = pt0[2]
    # rospy.loginfo(f"sensor origin {pt0} {pc2_points[-2, :]}")

    # t0 = rospy.Time.now()
    pts = copy.deepcopy(pc2_points)
    if z0 >= 0:
        bad_mask = pts[:, 2] >= z0
    else:
        bad_mask = pts[:, 2] <= z0
    z_scale = z0 / (z0 - pts[:, 2])
    pts[:, 0] = x0 + np.multiply(pts[:, 0] - x0, z_scale)
    pts[:, 1] = y0 + np.multiply(pts[:, 1] - y0, z_scale)
    pts[:, 2] = 0.0  # np.multiply(pts[:, 2] - z0, scale)
    # print(f"np {(rospy.Time.now() - t0).to_sec():0.6f}s")

    return pts, bad_mask


def points3d_to_plane(camera_to_target_transform: TransformStamped,
                      points3d_in_camera: np.ndarray,
                      points2d_in_camera: np.ndarray | None = None) -> ([Point], [np.ndarray], bool):
    """
    The final point in the input 3d array needs to be the sensor origin
    """
    # the camera origin in the target frame
    # rospy.loginfo(f"sensor origin {pt0} {pc2_points[-2, :]}")

    is_full = True
    points_in_plane = []
    # points2d_in_camera that intersected with the plane
    used_points2d_in_camera = []

    pts, bad_mask = points3d_to_plane_np(camera_to_target_transform, points3d_in_camera)
    # print(bad_mask)
    # TODO(lucasw) conversion to list of Point is slow
    for ind in range(pts.shape[0] - 1):
        if bad_mask[ind]:
            is_full = False
            continue
        x2 = pts[ind, 0]
        y2 = pts[ind, 1]
        z2 = pts[ind, 2]

        pt = Point(x2, y2, z2)
        # pt = Point(x1, y1, z1)
        points_in_plane.append(pt)
        if points2d_in_camera is not None:
            used_points2d_in_camera.append(points2d_in_camera[ind, :])

    return points_in_plane, used_points2d_in_camera, is_full


def points_in_camera_to_plane(points2d_in_camera: np.ndarray,
                              tf_buffer: tf2_ros.Buffer, camera_info: CameraInfo, target_frame: str,
                              ) -> ([Point], [np.ndarray], bool):
    try:
        tfs = tf_buffer.lookup_transform(target_frame, camera_info.header.frame_id,
                                         camera_info.header.stamp, rospy.Duration(0.3))
    except (tf2_ros.ConnectivityException, tf2_ros.LookupException, tf2_ros.ExtrapolationException) as ex:
        rospy.logwarn_throttle(4.0, ex)
        return None

    camera_matrix, dist_coeff, rvec, tvec = camera_info_to_cv2(camera_info)

    return points_in_camera_transform_to_plane(tfs, points2d_in_camera, camera_matrix, dist_coeff)


def points_to_marker(stamp: rospy.Time, frame: str, points: [Point], marker_id=0) -> Marker:
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame
    marker.ns = "camera_info"
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.pose.orientation.w = 1.0
    marker.lifetime = rospy.Duration(0.0)
    marker.frame_locked = False
    marker.scale.x = 0.05
    marker.color.r = 0.7
    marker.color.g = 1.0
    marker.color.a = 1.0
    marker.points.extend(points)
    return marker


def points_to_polygon(stamp: rospy.Time, frame: str, points: [Point]) -> PolygonStamped:
    polygon = PolygonStamped()
    polygon.header.stamp = stamp
    polygon.header.frame_id = frame
    for point in points:
        point32 = Point32(x=point.x, y=point.y, z=point.z)
        polygon.polygon.points.append(point32)
    return polygon


def camera_info_to_plane(tf_buffer: tf2_ros.BufferCore,
                         camera_info: CameraInfo,
                         target_frame: str,
                         output_frame: str,
                         camera_frame_override=None,
                         marker_in_camera_frame=True,
                         marker_id=0,
                         num_per_edge=8,
                         ) -> (Marker, TransformStamped, TransformStamped, bool):
    if camera_frame_override not in ["", None]:
        rospy.logwarn_once(f"overriding '{camera_info.header.frame_id}' with {camera_frame_override}")
        camera_info.header.frame_id = camera_frame_override

    try:
        tfs0 = tf_buffer.lookup_transform(target_frame, camera_info.header.frame_id,
                                          camera_info.header.stamp, rospy.Duration(0.3))
        camera_frame_to_target_plane_tfs = tfs0
        tfs1 = tf_buffer.lookup_transform(output_frame, target_frame,
                                          camera_info.header.stamp, rospy.Duration(0.3))
        target_to_output_tfs = tfs1
    except (tf2_ros.ConnectivityException, tf2_ros.LookupException, tf2_ros.ExtrapolationException,
            rospy.exceptions.ROSTimeMovedBackwardsException) as ex:
        rospy.logwarn_throttle(4.0, ex)
        return

    camera_matrix, dist_coeff, rvec, tvec = camera_info_to_cv2(camera_info)

    edge_points = get_camera_edge_points(camera_info, num_per_edge=num_per_edge)

    rv = points_in_camera_transform_to_plane(camera_frame_to_target_plane_tfs,
                                             edge_points, camera_matrix, dist_coeff)
    points_in_plane, _, is_full = rv

    if marker_in_camera_frame:
        try:
            tfs_inv = tf_buffer.lookup_transform(camera_info.header.frame_id, target_frame,
                                                 camera_frame_to_target_plane_tfs.header.stamp,
                                                 rospy.Duration(0.0))
        except (tf2_ros.ConnectivityException, tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                rospy.exceptions.ROSTimeMovedBackwardsException) as ex:
            rospy.logwarn_throttle(4.0, ex)
            return

        points_in_plane_in_camera = transform_points(points_in_plane, tfs_inv)

        marker = points_to_marker(camera_info.header.stamp, camera_info.header.frame_id,
                                  points_in_plane_in_camera, marker_id=marker_id)
    else:
        marker = points_to_marker(camera_info.header.stamp, target_frame,
                                  points_in_plane, marker_id=marker_id)

    marker.color.r -= marker_id / 5.0
    marker.color.b += marker_id / 8.0

    return marker, camera_frame_to_target_plane_tfs, target_to_output_tfs, is_full
