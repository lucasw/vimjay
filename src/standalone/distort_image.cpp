#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// from http://code.opencv.org/issues/1387
// TODO(lucasw) need to make a function that only returns the pixel_locations_dst,
// then the caller can re-use that instead of recomputing it if the intrinsics/dist
// doesn't change
void distort(const cv::Mat& src, cv::Mat& dst,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
  cv::Mat pixel_locations_src = cv::Mat(src.size(), CV_32FC2);

  for (int i = 0; i < src.size().height; i++) {
    for (int j = 0; j < src.size().width; j++) {
      pixel_locations_src.at<cv::Point2f>(i,j) = cv::Point2f(j,i);
    }
  }

  cv::Mat fractional_locations_dst = cv::Mat(src.size(), CV_32FC2);

  cv::Mat pixel_locations_dst = cv::Mat(src.size(), CV_32FC2);
  cv::undistortPoints(pixel_locations_src, pixel_locations_dst, cameraMatrix, distCoeffs);

  const float fx = cameraMatrix.at<double>(0,0);
  const float fy = cameraMatrix.at<double>(1,1);
  const float cx = cameraMatrix.at<double>(0,2);
  const float cy = cameraMatrix.at<double>(1,2);

  // is there a faster way to do this?
  for (int i = 0; i < fractional_locations_dst.size().height; i++) {
    for (int j = 0; j < fractional_locations_dst.size().width; j++) {
      const float x = fractional_locations_dst.at<cv::Point2f>(i,j).x*fx + cx;
      const float y = fractional_locations_dst.at<cv::Point2f>(i,j).y*fy + cy;
      pixel_locations_dst.at<cv::Point2f>(i,j) = cv::Point2f(x,y);
    }
  }

  cv::remap(src, dst, pixel_locations_dst, cv::Mat(), CV_INTER_LINEAR);
}

class DistortImage
{
public:
  DistortImage();
protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub_;
  void imageCallback(const sensor_msgs::ImageConstPtr msg);
  ros::Subscriber camera_info_sub_;
  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};

DistortImage::DistortImage() :
  it_(nh_)
{
  camera_matrix_ = cv::Mat(3, 3, CV_64F);
  image_pub_ = it_.advertise("distorted_image", 1, true);
  // image_sub_ = it_.subscribe("image", 1, &DistortImage::imageCallback, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &DistortImage::cameraInfoCallback, this);
}

void DistortImage::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
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
}

void DistortImage::imageCallback(const sensor_msgs::ImageConstPtr msg)
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

  cv::Mat distorted_cv_image;
  distort(cv_ptr->image, distorted_cv_image, camera_matrix_, dist_coeffs_);

  cv_bridge::CvImage distorted_image;
  distorted_image.header = msg->header;
  distorted_image.image = distorted_cv_image;
  image_pub_.publish(distorted_image.toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "distort_image");
  DistortImage distort_image;
  ros::spin();
}
