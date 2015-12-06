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
#include <vector>

class IirImage
{
protected:
  ros::NodeHandle nh_;
  // TODO(lucasw) or maybe capture N images then stop?
  image_transport::ImageTransport it_;
  // publish the most recent captured image
  image_transport::Publisher image_pub_;
  std::vector<image_transport::Subscriber> image_subs_;

  std::vector<double> b_coeffs_;
  // std::vector<cv::Mat> a_coeffs;
  // TODO(lucasw) maybe just temp debug
  // unsigned int index_;
  ros::Timer timer_;

  void pubImage(const ros::TimerEvent& e);

  // TODO(lucasw) or a deque of sensor_msgs/Images?
  std::vector<cv::Mat> in_frames_;
  std::deque<cv::Mat> out_frames_;
  bool dirty_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg, const size_t index);
public:
  IirImage();
};

IirImage::IirImage() :
  it_(nh_),
  dirty_(false)
{
  image_pub_ = it_.advertise("filtered_image", 1, true);

  ros::param::get("~b_coeffs", b_coeffs_);
  if (b_coeffs_.size() == 0)
  {
    ROS_WARN_STREAM("no b coefficients");
  }
  in_frames_.resize(b_coeffs_.size());
  for (size_t i = 0; i < b_coeffs_.size(); ++i)
  {
    std::stringstream ss;
    ss << "image_" << i;
    ROS_INFO_STREAM("subscribe " << ss.str() << " " << b_coeffs_[i]);
    image_subs_.push_back(it_.subscribe(ss.str(), 1,
                                        boost::bind(&IirImage::imageCallback, this, _1, i)));
  }

  timer_ = nh_.createTimer(ros::Duration(0.1), &IirImage::pubImage, this);
}

void IirImage::imageCallback(const sensor_msgs::ImageConstPtr& msg, const size_t index)
{
  if (index >= in_frames_.size())
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

  in_frames_[index] = cv_ptr->image.clone();
  dirty_ = true;
}

// TEMP code to show output of frames
void IirImage::pubImage(const ros::TimerEvent& e)
{
  if (!dirty_)
    return;

  cv::Mat out_frame;

  for (size_t i = 0; i < in_frames_.size() && i < b_coeffs_.size(); ++i)
  {
    const double bn = b_coeffs_[i];
    if (i == 0)
      out_frame = in_frames_[i] * bn;
    else if ((out_frame.size() == in_frames_[i].size()) &&
             (out_frame.type() == in_frames_[i].type()))
    {
      if (bn > 0)
        out_frame += in_frames_[i] * bn;
      else
        out_frame -= in_frames_[i] * -bn;
    }
  }

  {
    // TODO(lucasw) this may be argument for keeping original Image messages around
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now();  // or reception time of original message?
    cv_image.image = out_frame;
    cv_image.encoding = "rgb8";
    image_pub_.publish(cv_image.toImageMsg());
  }

  dirty_ = false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "iir_image");
  IirImage image_deque;
  ros::spin();
}
