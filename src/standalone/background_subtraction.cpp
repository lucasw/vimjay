/**
  Copyright 2016 Lucas Walter

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

Subtract the image background using opencv background subtraction methods,
also look at seamless cloning for adding images to background.

*/

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <vimjay/BackgroundSubtractionConfig.h>

class BackgroundSubtraction
{
protected:
  // TODO(lucasw) capture at a certain rate- and optionally force the images
  // to a certain rate even if at lower rate?
  // throttle tool already does first part, can combine that with capture_continuous.

  // bool writeImage();

  ros::NodeHandle nh_;
  // TODO(lucasw) or maybe capture N images then stop?
  image_transport::ImageTransport it_;
  // send a bool to indicate that an image was saved
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  // ros::Timer timer_;

  void pubImage(const ros::TimerEvent& e);

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  vimjay::BackgroundSubtractionConfig config_;
  typedef dynamic_reconfigure::Server<vimjay::BackgroundSubtractionConfig> ReconfigureServer;
  boost::recursive_mutex dr_mutex_;
  boost::shared_ptr<ReconfigureServer> server_;
  void callback(vimjay::BackgroundSubtractionConfig &config,
                uint32_t level);

  // foreground mask
  cv::Mat fg_mask_mog2_;
  cv::Mat fg_mask_mog2_rgb_;
  cv::Ptr<cv::BackgroundSubtractor> mog2_;

  cv::Mat bg_image_;
public:
  BackgroundSubtraction();
};

BackgroundSubtraction::BackgroundSubtraction():
  it_(nh_)
{
  mog2_ = cv::createBackgroundSubtractorMOG2();

  image_pub_ = it_.advertise("background_subtracted_image", 1, true);
  image_sub_ = it_.subscribe("image", 1, &BackgroundSubtraction::imageCallback, this);

  server_.reset(new ReconfigureServer(dr_mutex_, ros::NodeHandle(ros::this_node::getName())));
  dynamic_reconfigure::Server<vimjay::BackgroundSubtractionConfig>::CallbackType cbt =
    boost::bind(&BackgroundSubtraction::callback, this, boost::placeholders::_1, boost::placeholders::_2);
  server_->setCallback(cbt);

  // timer_ = nh_.createTimer(ros::Duration(0.2), &BackgroundSubtraction::pubImage, this);
}

void BackgroundSubtraction::callback(
    vimjay::BackgroundSubtractionConfig& config,
    uint32_t level)
{
  if (level & 1)
  {
    // if (config.capture_single)
    // {
    //   capture_single_ = true;
    //   config.capture_single = false;
    // }
  }
  config_ = config;
}

void BackgroundSubtraction::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
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

  cv::Mat live_frame = cv_ptr->image.clone();

  mog2_->apply(live_frame, fg_mask_mog2_);
  if (config_.capture_background)
  {
    bg_image_ = live_frame;
  }
  // else
  // if (config_.subtract_background)
  // if (!bg_image_.isEmpty())
  {
    cv::Mat image;
    #if 0
    cv::cvtColor(fg_mask_mog2_, fg_mask_mog2_rgb_, CV_GRAY2RGB);
    if (fg_mask_mog2_rgb_.size() == live_frame.size())
      image = live_frame & fg_mask_mog2_rgb_;
    else
      image = live_frame;
    #endif
    cv::Mat diff;
    cv::absdiff(bg_image_, live_frame, diff);
    cv::Mat diff_gray;
    cv::cvtColor(diff, diff_gray, CV_RGB2GRAY);
    cv::Mat mask = diff_gray > 20;  // TODO(lucasw) make this a config_ dr param
    cv::Mat mask_rgb;
    cv::cvtColor(mask, mask_rgb, CV_GRAY2RGB);
    image = mask_rgb & live_frame;

    cv_bridge::CvImage cv_image;
    cv_image.image = image;
    cv_image.encoding = "rgb8";
    cv_image.header.stamp = msg->header.stamp;
    cv_image.header.frame_id = msg->header.frame_id;
    image_pub_.publish(cv_image.toImageMsg());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "background_subtraction");
  BackgroundSubtraction background_subtraction;
  ros::spin();
}
