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


  Publish a bool based on keyboard input- hard code to space bar for now
*/

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "key_trigger");

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Bool>("trigger", 1);

  cv::Mat im(cv::Size(300, 300), CV_8UC3);
  im = cv::Scalar::all(128);
  cv::imshow(ros::this_node::getName(), im);
  /*
    Later want to read in a file with a list of keys and topics.
  */
  std_msgs::Bool msg;
  msg.data = true;
  while (ros::ok())
  {
    const int key = cv::waitKey(200);
    if (key == ' ')
      pub.publish(msg);
  }
}
