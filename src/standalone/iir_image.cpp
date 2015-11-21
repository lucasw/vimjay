/**
 Copyright 2015 Lucas Walter

     This file is part of Vimjay.

    Vimjay is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Vimjay is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Vimjay.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>

class ImageDeque
{
protected:

  // TODO(lucasw) capture at a certain rate- and optionally force the images
  // to a certain rate even if at lower rate?
  // throttle tool already does first part, can combine that with capture_continuous_.

  //bool writeImage();

  bool restrict_size_;

  ros::NodeHandle nh_;
  // TODO or maybe capture N images then stop?
  image_transport::ImageTransport it_;
  // publish the most recent captured image
  image_transport::Publisher captured_pub_;
  image_transport::Subscriber image_sub_;

  // TODO maybe just temp debug
  unsigned int index_;
  ros::Timer timer_;

  void pubImage(const ros::TimerEvent& e);

  // TODO(lucasw) or a deque of sensor_msgs/Images?
  std::deque<cv::Mat> frames_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  bool capture_single_;
  ros::Subscriber single_sub_;
  void singleCallback(const std_msgs::Bool::ConstPtr& msg);

  bool capture_continuous_;
  ros::Subscriber continuous_sub_;
  void continuousCallback(const std_msgs::Bool::ConstPtr& msg);

  unsigned int max_size_;
  ros::Subscriber max_size_sub_;
  void maxSizeCallback(const std_msgs::UInt16::ConstPtr& msg);

public:

  ImageDeque();
};

ImageDeque::ImageDeque() :
    it_(nh_),
    capture_single_(false),
    capture_continuous_(false),
    max_size_(10),
    restrict_size_(false),
    index_(0)
{
  captured_pub_ = it_.advertise("captured_image", 1, true);
  image_sub_ = it_.subscribe("image", 1, &ImageDeque::imageCallback, this);
  // TODO also dynamic reconfigure for these
  single_sub_ = nh_.subscribe<std_msgs::Bool>("single", 1,
      &ImageDeque::singleCallback, this);
  continuous_sub_ = nh_.subscribe<std_msgs::Bool>("continuous", 1,
      &ImageDeque::continuousCallback, this);
  max_size_sub_ = nh_.subscribe<std_msgs::UInt16>("max_size", 1,
      &ImageDeque::maxSizeCallback, this);

  timer_ = nh_.createTimer(ros::Duration(0.1), &ImageDeque::pubImage, this);
}

void ImageDeque::maxSizeCallback(const std_msgs::UInt16::ConstPtr& msg)
{
  max_size_ = msg->data;
}

void ImageDeque::singleCallback(const std_msgs::Bool::ConstPtr& msg)
{
  capture_single_ = msg->data;
}

void ImageDeque::continuousCallback(const std_msgs::Bool::ConstPtr& msg)
{
  capture_continuous_ = msg->data;
}

void ImageDeque::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!(capture_single_ || capture_continuous_))
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

  frames_.push_back(cv_ptr->image.clone());

  if (capture_single_)
    capture_single_ = false;

  if (restrict_size_)
  {
    while (frames_.size() > max_size_)
      // TODO also could have mode where it fills and then doesn't accept any more
      frames_.pop_front();
  }

  // TODO could put this in separate thread.
  // publish the exact same message received - TODO is this safe?
  //pub_.publish(msg);
}

// TEMP code to show output of frames
void ImageDeque::pubImage(const ros::TimerEvent& e)
{
  if (index_ < frames_.size())
  {
    // TODO this may be argument for keeping original Image messages around
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now(); // or reception time of original message?
    cv_image.image = frames_[index_];
    cv_image.encoding = "rgb8";
    captured_pub_.publish(cv_image.toImageMsg());
    index_++;
  }

  ROS_INFO_STREAM(frames_.size() << " " << index_);


  if (index_ >= frames_.size())
    index_ = 0;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_deque");
  ImageDeque image_deque;
  ros::spin();
}
