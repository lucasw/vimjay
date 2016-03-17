#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vimjay/cv_distort_image.h>

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
  cv::Mat map_1_;
  cv::Mat map_2_;
  bool new_maps_needed_;
};

DistortImage::DistortImage() :
  it_(nh_),
  use_debug_(false),
  new_maps_needed_(true)
{
  // TODO(lucasw) use CameraPublisher to sync camera info and image?
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  image_pub_ = it_.advertise("distorted/image", 1, true);
  ros::param::get("~use_debug", use_debug_);
  if (use_debug_)
    debug_image_pub_ = it_.advertise("debug_image", 1, true);
  camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("distorted/camera_info", 1);
  image_sub_ = it_.subscribe("image", 1, &DistortImage::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &DistortImage::cameraInfoCallback, this);
}

// TODO(lucasw) this use of a non synchronized callback is really non-standard
// should use a TimeSynchronizer and expect the camera info to have matching
// headers with the image.
void DistortImage::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_info_ = *msg;

  if (dist_coeffs_.rows != msg->D.size())
    new_maps_needed_ = true;
    dist_coeffs_ = cv::Mat(msg->D.size(), 1, CV_64F);

  for (size_t i = 0; i < msg->D.size(); ++i)
  {
    if (dist_coeffs_.at<double>(i, 0) != msg->D[i])
      new_maps_needed_ = true;
    dist_coeffs_.at<double>(i, 0) = msg->D[i];
  }

  int ind = 0;
  for (size_t y = 0; y < camera_matrix_.rows; ++y)
  {
    for (size_t x = 0; x < camera_matrix_.cols; ++x)
    {
      if (ind > msg->K.size())
      {
        // TODO(lucasw) save the old camera_info if the new
        // is bad?
        ROS_ERROR_STREAM(msg->K.size() << " " << camera_matrix_.size() << " " << ind);
        return;
      }

      if (camera_matrix_.at<double>(y, x) != msg->K[ind])
        new_maps_needed_ = true;
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

  cv::Size src_size = cv_ptr->image.size();
  cv::Mat distorted_cv_image;
  if (map_1_.empty() || map_2_.empty() ||
      (map_1_.size() != src_size) ||
      (map_2_.size() != src_size) ||
      new_maps_needed_)
  {
    initDistortMap(camera_matrix_, dist_coeffs_, src_size, map_1_, map_2_);
    new_maps_needed_ = false;
  }
  // Don't ever call distort() directly because it is more efficient to
  // reuse maps.
  // TODO(lucasw) make the interpolation controllable
  cv::remap(cv_ptr->image, distorted_cv_image, map_1_, map_2_, CV_INTER_LINEAR);

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
