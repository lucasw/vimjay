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
*/

#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <string>

class SaveImage
{
protected:
  ros::NodeHandle nh_;
  // TODO(lucasw) or maybe capture N images then stop?
  image_transport::ImageTransport it_;
  // publish the most recent captured image
  image_transport::Subscriber image_sub_;

  int counter_;
  int start_time_;
  std::string prefix_;

  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

public:
  SaveImage();
};

SaveImage::SaveImage() :
  it_(nh_),
  prefix_("frame")
{
  ros::param::get("~prefix", prefix_);

  std::stringstream ss;
  ss << "_" << int(ros::Time::now().toSec()) << "_";
  prefix_ += ss.str();

  image_sub_ = it_.subscribe("image", 1,
                             &SaveImage::imageCallback, this);
}

void SaveImage::imageCallback(const sensor_msgs::ImageConstPtr& msg)
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

  std::stringstream ss;
  ss << prefix_ << std::setw(8) << std::setfill('0') << counter_ << ".png";
  ROS_INFO_STREAM("saving " << ss.str());
  cv::imwrite(ss.str(), cv_ptr->image);
  counter_++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_image");
  SaveImage save_image;
  ros::spin();
}
