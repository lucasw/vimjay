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
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

class Noise
{
protected:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_;

  int width_;
  int height_;

  ros::Timer timer_;
  void pubImage(const ros::TimerEvent& e);

  int red_, green_, blue_;

public:
  Noise();
};

Noise::Noise() :
  it_(nh_),
  red_(128.0),
  green_(128),
  blue_(128),
  width_(360),
  height_(240)
{
  pub_ = it_.advertise("image", 1, true);

  ros::param::get("~red", red_);
  ros::param::get("~green", green_);
  ros::param::get("~blue", blue_);
  // this works okay but would rather get width and height from
  // an input image
  ros::param::get("~width", width_);
  ros::param::get("~height", height_);

  // TODO get width height
  timer_ = nh_.createTimer(ros::Duration(1.0), &Noise::pubImage, this);
  // pubImage(ros::TimerEvent());
}

void Noise::pubImage(const ros::TimerEvent& e)
{
  cv::Mat out = cv::Mat(cv::Size(width_, height_), CV_8UC3);
  out = cv::Scalar(red_, green_, blue_);

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now(); // or reception time of original message?
  cv_image.image = out;
  cv_image.encoding = "rgb8";
  pub_.publish(cv_image.toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "noise");
  Noise noise;
  ros::spin();
}
