#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

// from http://code.opencv.org/issues/1387
// TODO(lucasw) need to make a function that only returns the pixel_locations_dst,
// then the caller can re-use that instead of recomputing it if the intrinsics/dist
// doesn't change.
// Also would want to make use of convertMaps to make it even more efficient
void distort(const cv::Mat& src, cv::Mat& image_dst,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
  cv::Mat pixel_locations_src = cv::Mat(src.rows * src.cols, 1, CV_32FC2);

  int ind = 0;
  for (int i = 0; i < src.size().height; i++) {
    for (int j = 0; j < src.size().width; j++) {
      pixel_locations_src.at<cv::Point2f>(ind, 0) = cv::Point2f(j,i);
      ++ind;
    }
  }

  cv::Mat fractional_locations_dst = cv::Mat(pixel_locations_src.size(), CV_32FC2);
  cv::undistortPoints(pixel_locations_src, fractional_locations_dst, cameraMatrix, distCoeffs);

  const float fx = cameraMatrix.at<double>(0, 0);
  const float fy = cameraMatrix.at<double>(1, 1);
  const float cx = cameraMatrix.at<double>(0, 2);
  const float cy = cameraMatrix.at<double>(1, 2);

  // TODO(lucasw) is there a faster way to do this?
  // A matrix operation?
  cv::Mat pixel_locations_dst = cv::Mat(src.size(), CV_32FC2);
  ind = 0;
  for (int i = 0; i < src.size().height; i++) {
    for (int j = 0; j < src.size().width; j++) {
      const float x = fractional_locations_dst.at<cv::Point2f>(ind, 0).x * fx + cx;
      const float y = fractional_locations_dst.at<cv::Point2f>(ind, 0).y * fy + cy;
      pixel_locations_dst.at<cv::Point2f>(i,j) = cv::Point2f(x,y);
      // if ((i == 0) && (j == 0))
      //  ROS_INFO_STREAM(ind << ": " << i << " " << j << ", " << y << " " << x);
      ++ind;
    }
  }

  cv::remap(src, image_dst, pixel_locations_dst, cv::Mat(), CV_INTER_LINEAR);
}

class DistortImage
{
public:
  DistortImage();
protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  bool use_debug_;
  image_transport::Publisher debug_image_pub_;
  ros::Publisher camera_info_pub_;
  sensor_msgs::CameraInfo camera_info_;
  image_transport::Subscriber image_sub_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  ros::Subscriber camera_info_sub_;
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};

DistortImage::DistortImage() :
  it_(nh_),
  use_debug_(false)
{
  // TODO(lucasw) use CameraPublisher to sync camera info and image?
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  image_pub_ = it_.advertise("distorted/image", 1, true);
  if (use_debug_)
    debug_image_pub_ = it_.advertise("debug_image", 1, true);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("distorted/camera_info", 1);
  image_sub_ = it_.subscribe("image", 1, &DistortImage::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &DistortImage::cameraInfoCallback, this);
}

void DistortImage::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_info_ = *msg;
  dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);
  for (size_t i = 0; i < msg->D.size(); ++i)
  {
    dist_coeffs_.at<double>(i, 0) = msg->D[i];
  }

  int ind = 0;
  for (size_t y = 0; y < camera_matrix_.rows; ++y)
  {
    for (size_t x = 0; x < camera_matrix_.cols; ++x)
    {
      if (ind > msg->K.size())
      {
        ROS_ERROR_STREAM(msg->K.size() << " " << camera_matrix_.size() << " " << ind);
        return;
      }

      camera_matrix_.at<double>(y, x) = msg->K[ind];
      ++ind;
    }
  }

  // ROS_INFO_STREAM(dist_coeffs_);
}

void DistortImage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (camera_matrix_.empty())
    return;
  if (dist_coeffs_.empty())
    return;

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TBD why converting to BGR8
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    //, "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cv_ptr->image.empty())
  {
    ROS_ERROR("no converted image");
  }

  cv::Mat distorted_cv_image;
  distort(cv_ptr->image, distorted_cv_image, camera_matrix_, dist_coeffs_);

  cv_bridge::CvImage distorted_image;
  distorted_image.header = msg->header;
  distorted_image.encoding = msg->encoding;
  distorted_image.image = distorted_cv_image;
  sensor_msgs::ImagePtr image_msg = distorted_image.toImageMsg();
  camera_info_.header = image_msg->header;
  // camera_info_.roi.do_rectify = true;
  image_pub_.publish(image_msg);
  camera_info_pub_.publish(camera_info_);

  if (use_debug_)
  {
    cv::Mat debug_cv_image;
    cv::undistort(distorted_cv_image, debug_cv_image, camera_matrix_, dist_coeffs_);
    cv_bridge::CvImage debug_image;
    debug_image.header = msg->header;
    debug_image.encoding = msg->encoding;
    debug_image.image = debug_cv_image;
    debug_image_pub_.publish(debug_image.toImageMsg());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "distort_image");
  DistortImage distort_image;
  ros::spin();
}
