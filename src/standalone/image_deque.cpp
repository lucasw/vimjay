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
#include <dynamic_reconfigure/server.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <vimjay/ImageDequeConfig.h>

class ImageDeque
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
  ros::Publisher captured_trigger_pub_;
  // publish the most recent captured image
  image_transport::Publisher captured_pub_;
  image_transport::Publisher anim_pub_;
  image_transport::Subscriber image_sub_;

  // TODO(lucasw) maybe just temp debug
  unsigned int index_;
  ros::Timer timer_;

  void pubImage(const ros::TimerEvent& e);

  // this is for appending onto the animation output
  cv::Mat live_frame_;
  // TODO(lucasw) or a deque of sensor_msgs/Images?
  std::deque<cv::Mat> frames_;
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  bool capture_single_;
  ros::Subscriber single_sub_;
  void singleCallback(const std_msgs::Bool::ConstPtr& msg);

  ros::Subscriber continuous_sub_;
  void continuousCallback(const std_msgs::Bool::ConstPtr& msg);

  unsigned int max_size_;
  ros::Subscriber max_size_sub_;
  void maxSizeCallback(const std_msgs::UInt16::ConstPtr& msg);

  vimjay::ImageDequeConfig config_;
  typedef dynamic_reconfigure::Server<vimjay::ImageDequeConfig> ReconfigureServer;
  boost::recursive_mutex dr_mutex_;
  boost::shared_ptr<ReconfigureServer> server_;
  void callback(vimjay::ImageDequeConfig &config,
                uint32_t level);

public:
  ImageDeque();
};

ImageDeque::ImageDeque() :
  it_(nh_),
  capture_single_(false),
  index_(0)
{
  captured_trigger_pub_ = nh_.advertise<std_msgs::Bool>("captured_image_trigger", 1);
  captured_pub_ = it_.advertise("captured_image", 1, true);
  anim_pub_ = it_.advertise("anim", 1);
  image_sub_ = it_.subscribe("image", 1, &ImageDeque::imageCallback, this);
  // TODO(lucasw) also dynamic reconfigure for these
  single_sub_ = nh_.subscribe<std_msgs::Bool>("single", 1,
                &ImageDeque::singleCallback, this);
  continuous_sub_ = nh_.subscribe<std_msgs::Bool>("continuous", 1,
                    &ImageDeque::continuousCallback, this);
  max_size_sub_ = nh_.subscribe<std_msgs::UInt16>("max_size", 1,
                  &ImageDeque::maxSizeCallback, this);

  server_.reset(new ReconfigureServer(dr_mutex_, ros::NodeHandle(ros::this_node::getName())));
  dynamic_reconfigure::Server<vimjay::ImageDequeConfig>::CallbackType cbt =
    boost::bind(&ImageDeque::callback, this, _1, _2);
  server_->setCallback(cbt);

  timer_ = nh_.createTimer(ros::Duration(0.2), &ImageDeque::pubImage, this);
}

void ImageDeque::callback(
    vimjay::ImageDequeConfig& config,
    uint32_t level)
{
  if (level & 1)
  {
    if (config.capture_single)
    {
      capture_single_ = true;
      config.capture_single = false;
    }
  }
  if (level & 2)
  {
    if (config.update_rate > 0.0)
      timer_ = nh_.createTimer(ros::Duration(1.0 / config.update_rate),
          &ImageDeque::pubImage, this);
  }
  if (level & 4)
  {
    // TODO(lucasw) update config on every pubImage with latest index_?
    index_ = config.index;
  }
  if (level & 8)
  {
    if (config.start_index_to_end)
    {
      config.start_index = frames_.size();
      config.start_index_to_end = false;
    }

    if (config.pause)
    {
      if (config.next)
      {
        index_++;
      }
      if (config.prev)
      {
        ROS_INFO_STREAM("prev  " << index_);
        if (index_ > config.start_index)
        {
          index_--;
        }
        else if (frames_.size() > 0)
        {
          index_ = frames_.size() - 1;
        }
      }
      if (config.remove)
      {
        // TODO(lucasw) somehow need to move the saved frame from the save_image node also
        ROS_INFO_STREAM("remove " << index_ << " " << frames_.size());
        if (index_ < frames_.size())
        {
          frames_.erase(frames_.begin() + index_, frames_.begin() + index_ + 1);
        }
      }
    }
    config.next = false;
    config.prev = false;
    config.remove = false;
  }

  config_ = config;
  config.index = index_;
}

void ImageDeque::maxSizeCallback(const std_msgs::UInt16::ConstPtr& msg)
{
  // TODO(lwalter) update dr, also keep dr updated with current size
  config_.max_size = msg->data;
}

void ImageDeque::singleCallback(const std_msgs::Bool::ConstPtr& msg)
{
  capture_single_ = msg->data;
}

void ImageDeque::continuousCallback(const std_msgs::Bool::ConstPtr& msg)
{
  // TODO(lucasw) update dr
  config_.capture_continuous = msg->data;
}

void ImageDeque::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!config_.use_live_frame && (!(capture_single_ || config_.capture_continuous)))
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

  live_frame_ = cv_ptr->image.clone();

  if (!(capture_single_ || config_.capture_continuous))
    return;

  frames_.push_back(live_frame_);

  // TODO(lucasw) make this optional
  // save the image with unique timestamp from init time + counter
  // Or have node that just saves any image it receives?

  if (capture_single_)
  {
    // should capture single capture the one already received, rather
    // than here it captures the next one received?
    ROS_INFO_STREAM("capturing single");
    capture_single_ = false;
  }

  if (config_.restrict_size)
  {
    while (frames_.size() > config_.max_size)
      // TODO(lucasw) also could have mode where it fills and then doesn't accept any more
      frames_.pop_front();
  }

  // TODO(lucasw) could put this in separate thread.
  // publish the exact same message received - TODO(lucasw) is this safe?
  captured_pub_.publish(msg);
  std_msgs::Bool trigger;
  trigger.data = true;
  captured_trigger_pub_.publish(trigger);
}

// TEMP code to show output of frames
void ImageDeque::pubImage(const ros::TimerEvent& e)
{
  if (index_ <= frames_.size())
  {
    // TODO(lucasw) this may be argument for keeping original Image messages around
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = ros::Time::now();  // or reception time of original message?
    if (index_ < frames_.size())
      cv_image.image = frames_[index_];
    else
      // preview the live frame at the end of the saved animation
      cv_image.image = live_frame_;
    cv_image.encoding = "rgb8";
    anim_pub_.publish(cv_image.toImageMsg());

    if (!config_.pause)
    {
      index_++;
    }
  }

  // ROS_INFO_STREAM(frames_.size() << " " << index_);

  if ((config_.use_live_frame && (index_ > frames_.size())) ||
      (!config_.use_live_frame) && (index_ >= frames_.size()))
  {
    index_ = config_.start_index;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_deque");
  ImageDeque image_deque;
  ros::spin();
}
