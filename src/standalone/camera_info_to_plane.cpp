/*
 * Lucas Walter
 * Adaptation of camera_info_to_plane.py
 */


#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/TransformStamped.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// originally adapted from https://github.com/ros-perception/vision_opencv/blob/noetic/image_geometry/src/
// pinhole_camera_model.cpp fromCameraInfo
// copied from image_manip utility.cpp
void cameraInfoToCV(const sensor_msgs::CameraInfo& camera_info,
    cv::Matx33d& K,  // Describe current image (includes binning, ROI)
    cv::Mat_<double>& D)  // Unaffected by binning, ROI - they are in ideal camera coordinates
{
  cv::Matx34d P;  // Describe current image (includes binning, ROI)

  const size_t d_size = camera_info.D.size();
  // TODO(lucasw) initializing like this doesn't work, only coincidence when memory happened to be all zeros
  // and it appeared to work
  // D = (d_size == 0) ? cv::Mat_<double>() : cv::Mat_<double>(1, d_size, camera_info.D.data());
  if (d_size > 0) {
    D = cv::Mat_<double>(1, d_size);
    for (size_t i = 0; i < d_size; ++i) {
      D.at<double>(0, i) = camera_info.D[i];
    }
  } else {
    D = cv::Mat_<double>();
  }
  auto K_full = cv::Matx33d(&camera_info.K[0]);
  // TODO(lucasw) not actually using P_full_
  auto P_full = cv::Matx34d(&camera_info.P[0]);

  // Binning = 0 is considered the same as binning = 1 (no binning).
  const uint32_t binning_x = camera_info.binning_x ? camera_info.binning_x : 1;
  const uint32_t binning_y = camera_info.binning_y ? camera_info.binning_y : 1;

  // ROI all zeros is considered the same as full resolution.
  sensor_msgs::RegionOfInterest roi = camera_info.roi;
  if (roi.x_offset == 0 && roi.y_offset == 0 && roi.width == 0 && roi.height == 0) {
    roi.width  = camera_info.width;
    roi.height = camera_info.height;
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

// TODO(lucasw) PointCloud is only 32-bit xyz, want double
sensor_msgs::PointCloud transform_points(
    const sensor_msgs::PointCloud& cloud_in,
    const geometry_msgs::TransformStamped& tfs)
{
  sensor_msgs::PointCloud2 cloud2_in;
  sensor_msgs::convertPointCloudToPointCloud2(cloud_in, cloud2_in);

  sensor_msgs::PointCloud2 cloud2_out;
  tf2::doTransform(cloud2_in, cloud2_out, tfs);

  sensor_msgs::PointCloud cloud_out;
  sensor_msgs::convertPointCloud2ToPointCloud(cloud2_out, cloud_out);
  return cloud_out;
}

std::vector<cv::Point3f> transform_points(
    const std::vector<cv::Point3f>& points_in,
    const geometry_msgs::TransformStamped& tfs)
{
  sensor_msgs::PointCloud cloud_in;
  for (const auto& pt : points_in) {
    geometry_msgs::Point32 gpt;
    gpt.x = pt.x;
    gpt.y = pt.y;
    gpt.z = pt.z;
    cloud_in.points.push_back(gpt);
  }

  const auto cloud_out = transform_points(cloud_in, tfs);

  std::vector<cv::Point3f> points_out;
  for (const auto& pt : cloud_out.points) {
    cv::Point3f gpt;
    gpt.x = pt.x;
    gpt.y = pt.y;
    gpt.z = pt.z;
    points_out.push_back(gpt);
  }

  return points_out;
}

bool points_in_camera_transform_to_plane(
    const geometry_msgs::TransformStamped& camera_to_target_transform,
    const std::vector<cv::Point2f> points2d_in_camera,
    const cv::Matx33d& camera_matrix,
    const cv::Mat_<double>& dist_coeff,
    std::vector<cv::Point3f>& points_in_plane,
    std::vector<cv::Point2f>& used_points2d_in_camera,
    sensor_msgs::PointCloud& pc_points)
{
  // idealized points all at range 1.0
  const size_t num_points = points2d_in_camera.size();
  // cv::Mat ideal_points = cv::Mat(num_points, CV_32FC2);
  std::vector<cv::Point2f> ideal_points;
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

  // const auto
  pc_points = transform_points(cloud_in_camera, camera_to_target_transform);

  // the camera origin in the target frame
  const auto pt0 = pc_points.points[num_points];
  const double x0 = pt0.x;
  const double y0 = pt0.y;
  const double z0 = pt0.z;

  bool is_full = true;

  // TODO(lucasw) replace for loop with matrix operation
  for (size_t ind = 0; ind < num_points; ++ind) {
    const double x1 = pc_points.points[ind].x;
    const double y1 = pc_points.points[ind].y;
    const double z1 = pc_points.points[ind].z;
    // is the ray facing away from the plane, or parallel, and will never intersect?
    const bool non_intersecting = (z1 >= z0 and z0 >= 0.0) or (z1 >= z0 and z0 <= 0.0);
    if (non_intersecting) {
      is_full = false;
      ROS_WARN_STREAM_THROTTLE(8.0, "non intersecting points");
      continue;
    }

    const auto intersect_distance = z0 / (z0 - z1);
    // the point intersecting the plane
    const auto x2 = x0 + (x1 - x0) * intersect_distance;
    const auto y2 = y0 + (y1 - y0) * intersect_distance;
    // this should be 0.0
    const auto z2 = z0 + (z1 - z0) * intersect_distance;

    points_in_plane.push_back(cv::Point3f(x2, y2, z2));
    used_points2d_in_camera.push_back(points2d_in_camera[ind]);
  }

  return is_full;
}

bool points_in_camera_to_plane(const std::vector<cv::Point2f>& points2d_in_camera,
                               const tf2_ros::Buffer& tf_buffer,
                               const sensor_msgs::CameraInfo& camera_info,
                               const std::string& target_frame,
                               std::vector<cv::Point3f>& points_in_plane,
                               std::vector<cv::Point2f>& used_points2d_in_camera,
                               sensor_msgs::PointCloud& point_cloud)
{
  geometry_msgs::TransformStamped tfs;
  try {
    tfs = tf_buffer.lookupTransform(target_frame, camera_info.header.frame_id,
                                    camera_info.header.stamp, ros::Duration(0.3));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_STREAM_THROTTLE(4.0, ex.what());
    return false;
  }

  cv::Matx33d camera_matrix;
  cv::Mat_<double> dist_coeff;
  cameraInfoToCV(camera_info, camera_matrix, dist_coeff);

  return points_in_camera_transform_to_plane(
      tfs, points2d_in_camera,
      camera_matrix, dist_coeff,
      points_in_plane,
      used_points2d_in_camera,
      point_cloud);
}

visualization_msgs::Marker points_to_marker(const ros::Time& stamp, const std::string& frame,
    const std::vector<cv::Point3f>& points, const int marker_id=0)
{
  visualization_msgs::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame;
  marker.ns = "camera_info";
  marker.id = marker_id;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = ros::Duration(0.0);
  marker.frame_locked = false;
  marker.scale.x = 0.05;
  marker.color.r = 0.7;
  marker.color.g = 1.0;
  marker.color.a = 1.0;
  for (const auto& pt : points) {
    geometry_msgs::Point gpt;
    gpt.x = pt.x;
    gpt.y = pt.y;
    gpt.z = pt.z;
    marker.points.push_back(gpt);
  }
  return marker;
}

bool camera_info_to_plane(const tf2_ros::Buffer& tf_buffer,
                          sensor_msgs::CameraInfo camera_info,
                          const std::string& target_frame,
                          const std::string& output_frame,
                          visualization_msgs::Marker& marker,
                          geometry_msgs::TransformStamped& camera_frame_to_target_plane_tfs,
                          bool& is_full,
                          sensor_msgs::PointCloud& point_cloud,
                          const std::string camera_frame_override="",
                          const bool marker_in_camera_frame=true,
                          const int marker_id=0,
                          const size_t num_per_edge=8)
{
  if (camera_frame_override != "") {
    ROS_WARN_STREAM_ONCE("overriding " << camera_info.header.frame_id << " with " << camera_frame_override);
    camera_info.header.frame_id = camera_frame_override;
  }

  geometry_msgs::TransformStamped target_to_output_tfs;
  try {
    auto tfs0 = tf_buffer.lookupTransform(target_frame, camera_info.header.frame_id,
                                          camera_info.header.stamp, ros::Duration(0.3));
    camera_frame_to_target_plane_tfs = tfs0;
    auto tfs1 = tf_buffer.lookupTransform(output_frame, target_frame,
                                          camera_info.header.stamp, ros::Duration(0.3));
    target_to_output_tfs = tfs1;
  } catch (tf2::TransformException& ex) {
    ROS_WARN_STREAM_THROTTLE(4.0, ex.what());
    return false;
  }
  // rospy.exceptions.ROSTimeMovedBackwardsException) as ex:

  cv::Matx33d camera_matrix;
  cv::Mat_<double> dist_coeff;
  cameraInfoToCV(camera_info, camera_matrix, dist_coeff);

  auto edge_points = get_camera_edge_points(camera_info, num_per_edge);

  std::vector<cv::Point3f> points_in_plane;
  std::vector<cv::Point2f> used_points2d_in_camera;
  try {
    is_full = points_in_camera_transform_to_plane(camera_frame_to_target_plane_tfs,
                                                  edge_points, camera_matrix, dist_coeff,
                                                  points_in_plane, used_points2d_in_camera,
                                                  point_cloud);
  } catch (cv::Exception& ex) {
    ROS_WARN_STREAM(ex.what());
    return false;
  }

  if (marker_in_camera_frame) {
    geometry_msgs::TransformStamped tfs_inv;
    try {
      tfs_inv = tf_buffer.lookupTransform(camera_info.header.frame_id, target_frame,
                                          camera_frame_to_target_plane_tfs.header.stamp,
                                          ros::Duration(0.0));
    } catch (tf2::TransformException& ex) {
      ROS_WARN_STREAM_THROTTLE(4.0, ex.what());
      return false;
    }

    const auto points_in_plane_in_camera = transform_points(points_in_plane, tfs_inv);

    marker = points_to_marker(camera_info.header.stamp, camera_info.header.frame_id,
                              points_in_plane_in_camera, marker_id);
  } else {
    marker = points_to_marker(camera_info.header.stamp, target_frame,
                              points_in_plane, marker_id);

    marker.color.r -= marker_id / 5.0;
    marker.color.b += marker_id / 8.0;
  }

  return true;
}

class CameraInfoToPlane
{
  ros::NodeHandle nh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher marker_pub_;
  ros::Publisher point_cloud_pub_;
  ros::Subscriber ci_sub_;

  std::string target_frame_;
  std::string output_frame_ = "";

  std::string camera_frame_override_ = "";
  bool marker_in_camera_frame_ = true;
  int marker_id_ = 0;
  std::string marker_ns_ = "camera_info";
  int num_per_edge_ = 8;

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    visualization_msgs::Marker marker;
    geometry_msgs::TransformStamped camera_frame_to_target_plane_tfs;
    bool is_full;
    sensor_msgs::PointCloud point_cloud;
    const bool rv = camera_info_to_plane(
        tf_buffer_,
        *msg,
        target_frame_,
        output_frame_,
        marker,
        camera_frame_to_target_plane_tfs,
        is_full,
        point_cloud,
        camera_frame_override_,
        marker_in_camera_frame_,
        marker_id_,
        num_per_edge_);
    if (!rv) {
      return;
    }

    marker.ns = marker_ns_;
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    marker_pub_.publish(marker_array);
    point_cloud_pub_.publish(point_cloud);
  }

public:
  CameraInfoToPlane() :
      tf_buffer_(ros::Duration(20.0)),
      tf_listener_(tf_buffer_)
  {
    ros::param::get("~target_frame", target_frame_);
    ros::param::get("~output_frame", output_frame_);
    if (output_frame_ == "") {
      output_frame_ = target_frame_;
    }
    ROS_WARN_STREAM("target frame " << target_frame_ << ", output frame " << output_frame_);

    ros::param::get("~marker_id", marker_id_);
    ros::param::get("~marker_ns", marker_ns_);
    ros::param::get("~marker_in_camera_frame", marker_in_camera_frame_);
    ros::param::get("~num_per_edge", num_per_edge_);
    ros::param::get("~camera_frame_override", camera_frame_override_);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_array", 3);
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud>("point_cloud", 3);
    ci_sub_ = nh_.subscribe("camera_info", 2, &CameraInfoToPlane::cameraInfoCallback, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_info_to_plane");
  CameraInfoToPlane camera_info_to_plane;
  ros::spin();
  return 0;
}
