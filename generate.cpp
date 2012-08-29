/*
  
  Copyright 2012 Lucas Walter

     This file is part of Camthing.

    Camthing is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Camthing is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Camthing.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "generate.h"
#include "camthing.h"
#include <glog/logging.h>

// filter Node objects
namespace bm {

Bezier::Bezier()
{
  setSignal("x0",10);
  setSignal("y0",10);
  
  setSignal("x1",100);
  setSignal("y1",50);
  
  setSignal("x2",200);
  setSignal("y2",50);
  
  setSignal("x3",300);
  setSignal("y3",150);

  setSignal("num", 4);
}

bool Bezier::update()
{
  //if (!ImageNode::update()) return false;
  if (!Node::update()) return false;

  cv::Mat out = cv::Mat(cv::Size(Config::inst()->im_width, Config::inst()->im_height), MAT_FORMAT_C3);
  out = cv::Scalar(0,0,0);

  std::vector<cv::Point2f> control_points;
  control_points.push_back( cv::Point2f( getSignal("x0"), getSignal("y0") ));
  control_points.push_back( cv::Point2f( getSignal("x1"), getSignal("y1") ));
  control_points.push_back( cv::Point2f( getSignal("x2"), getSignal("y2") ));
  control_points.push_back( cv::Point2f( getSignal("x3"), getSignal("y3") ));
  
  int num = getSignal("num");
  if (num < 2) { num = 2;
    setSignal("num", num);
  }

  std::vector<cv::Point2f> bezier_points;
  getBezier(control_points, bezier_points, num);

  for (int i = 1; i < bezier_points.size(); i++) {
    cv::line(out, bezier_points[i-1], bezier_points[i], cv::Scalar(255, 255, 255), 2, CV_AA ); 
  }

  setImage("out", out);

  return true;
}


} // namespace bm

