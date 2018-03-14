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

  Save a received image to disk

  TODO(lucasw) want to also have a gate_image node which passes images
  through when triggered- but for now image_deque and this node will
  duplicate that functionality and only save or add to the deque 
  when triggered.
*/

#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <string>

class SaveImage
{
protected:
  ros::NodeHandle nh_;
  // TODO(lucasw) or maybe capture N images then stop?
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher saved_pub_;

  int counter_;
  int start_time_;
  std::string prefix_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  // maybe these should be in base class
  bool capture_single_;
  ros::Subscriber single_sub_;
  void singleCallback(const std_msgs::Bool::ConstPtr& msg);

  bool capture_continuous_;
  ros::Subscriber continuous_sub_;
  void continuousCallback(const std_msgs::Bool::ConstPtr& msg);

public:
  SaveImage();
};

SaveImage::SaveImage() :
  it_(nh_),
  prefix_("frame"),
  capture_single_(false),
  capture_continuous_(false),
  counter_(0)
{
  ros::param::get("~prefix", prefix_);
  ros::param::get("~capture_continuous", capture_continuous_);

  saved_pub_ = it_.advertise("saved_image", 1, true);

  std::stringstream ss;
  ss << "_" << int(ros::Time::now().toSec()) << "_";
  prefix_ += ss.str();

  single_sub_ = nh_.subscribe<std_msgs::Bool>("single", 1,
                &SaveImage::singleCallback, this);
  continuous_sub_ = nh_.subscribe<std_msgs::Bool>("continuous", 1,
                    &SaveImage::continuousCallback, this);

  image_sub_ = it_.subscribe("image", 1,
                             &SaveImage::imageCallback, this);
}

void SaveImage::singleCallback(const std_msgs::Bool::ConstPtr& msg)
{
  capture_single_ = msg->data;
}

void SaveImage::continuousCallback(const std_msgs::Bool::ConstPtr& msg)
{
  capture_continuous_ = msg->data;
}

void SaveImage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!(capture_single_ || capture_continuous_))
    return;
  capture_single_ = false;

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    // TBD why converting to BGR8
    // cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    // , "mono8"); // sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  std::stringstream ss;
  ss << prefix_ << std::setw(8) << std::setfill('0') << counter_ << ".png";
  ROS_INFO_STREAM("saving " << ss.str());
  cv::imwrite(ss.str(), cv_ptr->image);
  counter_++;

  saved_pub_.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_image");
  SaveImage save_image;
  ros::spin();
}
