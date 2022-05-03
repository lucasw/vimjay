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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

class CamSub
{
protected:
  bool restrict_size_;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_1_;
  image_transport::CameraSubscriber sub_2_;
  image_transport::Subscriber sub_3_;

  void imageCallback1(const sensor_msgs::ImageConstPtr& msg,
      const sensor_msgs::CameraInfoConstPtr& ci,
      const int ind);

  void imageCallback2(const sensor_msgs::ImageConstPtr& msg,
      const sensor_msgs::CameraInfoConstPtr& ci);

  void imageCallback3(const sensor_msgs::ImageConstPtr& msg,
      const int ind);
public:
  CamSub();
};

CamSub::CamSub() :
  it_(nh_)
{
  const uint32_t queue_size = 1;
  const int ind = 5;
  sub_1_ = it_.subscribeCamera(
      "image",
      queue_size,
      boost::bind(&CamSub::imageCallback1, this, boost::placeholders::_1, boost::placeholders::_2, ind));
  sub_2_ = it_.subscribeCamera(
      "image",
      queue_size,
      &CamSub::imageCallback2,
      this);
  sub_3_ = it_.subscribe(
      "image",
      queue_size,
      boost::bind(&CamSub::imageCallback3, this, boost::placeholders::_1, ind));
}

void CamSub::imageCallback1(const sensor_msgs::ImageConstPtr& msg,
      const sensor_msgs::CameraInfoConstPtr& ci, const int ind)
{
}

void CamSub::imageCallback2(const sensor_msgs::ImageConstPtr& msg,
      const sensor_msgs::CameraInfoConstPtr& ci)
{
}

void CamSub::imageCallback3(const sensor_msgs::ImageConstPtr& msg,
      const int ind)
{
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cam_sub");
  CamSub image_deque;
  ros::spin();
}
