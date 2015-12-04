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

  unsigned int type_;
  unsigned int width_;
  unsigned int height_;

  float mean_;
  ros::Subscriber mean_sub_;
  void meanCallback(const std_msgs::Float32::ConstPtr& msg);

  float stddev_;
  ros::Subscriber stddev_sub_;
  void stddevCallback(const std_msgs::Float32::ConstPtr& msg);

  ros::Timer timer_;
  void pubImage(const ros::TimerEvent& e);

public:
  Noise();
};

Noise::Noise() :
  it_(nh_),
  mean_(10.0),
  stddev_(128),
  width_(360),
  height_(240)
{
  pub_ = it_.advertise("image", 1, true);
  mean_sub_ = nh_.subscribe<std_msgs::Float32>("mean", 1,
              &Noise::meanCallback, this);
  stddev_sub_ = nh_.subscribe<std_msgs::Float32>("stddev", 1,
                &Noise::stddevCallback, this);

  // TODO get width height
  timer_ = nh_.createTimer(ros::Duration(0.1), &Noise::pubImage, this);
}

void Noise::meanCallback(const std_msgs::Float32::ConstPtr& msg)
{
  mean_ = msg->data;
}

void Noise::stddevCallback(const std_msgs::Float32::ConstPtr& msg)
{
  stddev_ = msg->data;
}

void Noise::pubImage(const ros::TimerEvent& e)
{
  cv::Mat out = cv::Mat(cv::Size(width_, height_), CV_8UC3);

  int type = 1;
  if (type == 0)
  {
    // uniform
    cv::randu(out, cv::Scalar(0, 0, 0, 0), cv::Scalar(255, 255, 255, 255));
  }
  else
  {
    // TBD handle alpha channel
    cv::randn(out, cv::Scalar(mean_, mean_, mean_, mean_),
              cv::Scalar(stddev_, stddev_, stddev_, stddev_));
  }

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
