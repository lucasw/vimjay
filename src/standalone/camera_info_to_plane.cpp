/*
 * Lucas Walter
 * Adaptation of camera_info_to_plane.py
 */


#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Header.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/Marker.h>


// originally adapted from https://github.com/ros-perception/vision_opencv/blob/noetic/image_geometry/src/
// pinhole_camera_model.cpp fromCameraInfo
// copied from image_manip utility.cpp
void cameraInfoToCV(const sensor_msgs::CameraInfo::ConstPtr& msg,
    cv::Matx33d& K,  // Describe current image (includes binning, ROI)
    cv::Mat_<double>& D)  // Unaffected by binning, ROI - they are in ideal camera coordinates
{
  // TODO(lucasw) this can't be const
  auto cam_info = *msg;

  cv::Matx34d P;  // Describe current image (includes binning, ROI)

  const size_t d_size = cam_info.D.size();
  // TODO(lucasw) initializing like this doesn't work, only coincidence when memory happened to be all zeros
  // and it appeared to work
  // D = (d_size == 0) ? cv::Mat_<double>() : cv::Mat_<double>(1, d_size, cam_info.D.data());
  if (d_size > 0) {
    D = cv::Mat_<double>(1, d_size);
    for (size_t i = 0; i < d_size; ++i) {
      D.at<double>(0, i) = cam_info.D[i];
    }
  } else {
    D = cv::Mat_<double>();
  }
  auto K_full = cv::Matx33d(&cam_info.K[0]);
  // TODO(lucasw) not actually using P_full_
  auto P_full = cv::Matx34d(&cam_info.P[0]);

  // Binning = 0 is considered the same as binning = 1 (no binning).
  const uint32_t binning_x = cam_info.binning_x ? cam_info.binning_x : 1;
  const uint32_t binning_y = cam_info.binning_y ? cam_info.binning_y : 1;

  // ROI all zeros is considered the same as full resolution.
  sensor_msgs::RegionOfInterest roi = cam_info.roi;
  if (roi.x_offset == 0 && roi.y_offset == 0 && roi.width == 0 && roi.height == 0) {
    roi.width  = cam_info.width;
    roi.height = cam_info.height;
  }

  // If necessary, create new K and P adjusted for binning and ROI

  /// @todo Calculate and use rectified ROI
  const bool adjust_binning = (binning_x > 1) || (binning_y > 1);
  const bool adjust_roi = (roi.x_offset != 0) || (roi.y_offset != 0);

  if (!adjust_binning && !adjust_roi) {
    K = K_full;
    P = P_full;
  } else {
    K = K_full;
    P = P_full;

    // ROI is in full image coordinates, so change it first
    if (adjust_roi) {
      // Move principal point by the offset
      /// @todo Adjust P by rectified ROI instead
      K(0, 2) -= roi.x_offset;
      K(1, 2) -= roi.y_offset;
      P(0, 2) -= roi.x_offset;
      P(1, 2) -= roi.y_offset;
    }

    if (binning_x > 1) {
      const double scale_x = 1.0 / binning_x;
      K(0, 0) *= scale_x;
      K(0, 2) *= scale_x;
      P(0, 0) *= scale_x;
      P(0, 2) *= scale_x;
      P(0, 3) *= scale_x;
    }
    if (binning_y > 1) {
      const double scale_y = 1.0 / binning_y;
      K(1, 1) *= scale_y;
      K(1, 2) *= scale_y;
      P(1, 1) *= scale_y;
      P(1, 2) *= scale_y;
      P(1, 3) *= scale_y;
    }
  }
}

std::vector<cv::Point2f> get_camera_edge_points(const sensor_msgs::CameraInfo& camera_info,
    const size_t num_per_edge=8)
{
  const size_t num_points = num_per_edge * 4 + 1;

  std::vector<cv::Point2f> points2d;
  points2d.resize(num_points);
  size_t ind = 0;
  // create points in a loop around the edge of the image
  for (size_t i = 0; i < num_per_edge; ++i) {
    const float fr = i / num_per_edge;
    points2d[ind].x = fr * camera_info.width;
    ind += 1;
  }

  for (size_t i = 0; i < num_per_edge; ++i) {
    const float fr = i / num_per_edge;
    points2d[ind].x = static_cast<float>(camera_info.width);
    points2d[ind].y = fr * camera_info.height;
    ind += 1;
  }

  for (size_t i = 0; i < num_per_edge; ++i) {
    const float fr = 1.0 - i / num_per_edge;
    points2d[ind].x = fr * camera_info.width;
    points2d[ind].y = static_cast<float>(camera_info.height);
    ind += 1;
  }

  // complete the loop with the + 1
  for (size_t i = 0; i < num_per_edge + 1; ++i) {
    const float fr = 1.0 - i / num_per_edge;
    points2d[ind].y = fr * camera_info.height;
    ind += 1;
  }
  return points2d;
}

#if 0
// TODO(lucasw) is there a an off-the-shelf function that does this?
 transform_points_np(points3d: np.ndarray, transform: TransformStamped) -> np.ndarray:
    ""
    points3d is n x 3 in the src_header.frame_id frame
    transform child frame id should match src_header frame id
    transform header frame id is the destination frame
    ""
    # put numpy/cv2 points into a point cloud
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

    src_header = Header()
    src_header.frame_id = transform.child_frame_id
    src_header.stamp = transform.header.stamp
    pc2_in = point_cloud2.create_cloud(src_header, fields, field_points)
    # pc2_in.header.stamp = camera_info.header.stamp
    # TODO(lucasw) probably a more efficient transform on a Point array than converting to PointCloud2
    pc2_out = do_transform_cloud(pc2_in, transform)
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

#endif

bool points_in_camera_transform_to_plane(
    const geometry_msgs::TransformStamped& camera_to_target_transform,
    const std::vector<cv::Point2f> points2d_in_camera,
    const cv::Matx33d& camera_matrix,
    const cv::Mat_<double>& dist_coeff,
    std::vector<cv::Point2f>& points_in_plane)
{
  // idealized points all at range 1.0
  const size_t num_points = points2d_in_camera.size();
  // cv::Mat ideal_points = cv::Mat(num_points, CV_32FC2);
  std::vector<cv::Point3f> ideal_points;
  cv::undistortPoints(points2d_in_camera, ideal_points, camera_matrix, dist_coeff);

  sensor_msgs::PointCloud cloud_in_camera;
  cloud_in_camera.points.resize(num_points + 1);
  for (size_t i = 0; i < num_points; ++i) {
    cloud_in_camera.points[i].x = ideal_points[i].x;
    cloud_in_camera.points[i].y = ideal_points[i].y;
    cloud_in_camera.points[i].z = 1.0;
  }

  // the origin of the camera
  cloud_in_camera.points[num_points].x = 0.0;
  cloud_in_camera.points[num_points].y = 0.0;
  cloud_in_camera.points[num_points].z = 0.0;

  // rospy.loginfo_throttle(6.0, f"\n{points2d_in_camera}")
  // rospy.loginfo_throttle(6.0, f"\n{cloud_in_camera}")
  sensor_msgs::PointCloud2 cloud_in_camera2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud_in_camera, cloud_in_camera2);

  sensor_msgs::PointCloud2 cloud_out;
  tf2::doTransform(cloud_in_camera2, cloud_out, camera_to_target_transform);
#if 0
    pc2_points = transform_points_np(points3d_in_camera, camera_to_target_transform)

    # the camera origin in the target frame
    pt0 = pc2_points[-1, :]
    x0 = pt0[0]
    y0 = pt0[1]
    z0 = pt0[2]

    is_full = True
    points_in_plane = []
    # points2d_in_camera that intersected with the plane
    used_points2d_in_camera = []
    for ind in range(pc2_points.shape[0] - 1):
        x1 = pc2_points[ind, 0]
        y1 = pc2_points[ind, 1]
        z1 = pc2_points[ind, 2]
        # is the ray facing away from the plane, or parallel, and will never intersect?
        non_intersecting = (z1 >= z0 and z0 >= 0.0) or (z1 >= z0 and z0 <= 0.0)
        if non_intersecting:
            is_full = False
            continue

        scale = z0 / (z0 - z1)
        x2 = x0 + (x1 - x0) * scale
        y2 = y0 + (y1 - y0) * scale
        # this should be 0.0
        z2 = z0 + (z1 - z0) * scale

        pt = Point(x2, y2, z2)
        # pt = Point(x1, y1, z1)
        points_in_plane.append(pt)
        used_points2d_in_camera.append(points2d_in_camera[ind, :])

    return points_in_plane, used_points2d_in_camera, is_full
#endif
  return true;
}

#if 0
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

#endif

int main()
{
  return 0;
}
