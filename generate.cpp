/*
  
  Copyright 2012 Lucas Walter

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


#include "generate.h"
#include "config.h"
#include <glog/logging.h>

// filter Node objects
namespace bm {

Bezier::Bezier(const std::string name) : ImageNode(name)
{
  // TBD take x and y sigbuf inputs
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

//////////////////////////////////////////////////

Circle::Circle(const std::string name) : ImageNode(name)
{
  // hard to enforce these all being the same size
  //setSigBuf("x");
  //setSigBuf("y");
  //setSigBuf("r");
  setSignal("x", 300);
  setSignal("y", 300);
  setSignal("radius", 50);
  setSignal("r", 255);
  setSignal("g", 255);
  setSignal("b", 255);
  setSignal("thickness", -1);
}

bool Circle::update()
{
  if (!Node::update()) return false;


  // if any inputs have changed this will go on to draw
  const bool id1 = isDirty(this,30);
  //LOG(INFO) << id1;
  if (!id1) {
    return true;
  }

  cv::Mat out = cv::Mat(cv::Size(Config::inst()->im_width, Config::inst()->im_height), 
      MAT_FORMAT_C3);
  out = cv::Scalar(0,0,0,0);
 
  cv::circle(out, 
    cv::Point(getSignal("x"), getSignal("y")), 
    getSignal("radius"), 
    cv::Scalar(
      getSignal("b"),
      getSignal("g"),
      getSignal("r")),
    getSignal("thickness")); 
  
  setImage("out", out);
  // clear this isDirty in advance of next loop
  const bool id2 = isDirty(this,30);
  const bool id3 = isDirty(this,30);
  //LOG(INFO) << id2 << " " << id3;
}

//////////////////////////////////////////////////
Noise::Noise(const std::string name) : ImageNode(name)
{
  setSignal("mean", 10);
  setSignal("stddev", 128);
}

bool Noise::update()
{
  if (!Node::update()) return false;
  
  cv::Mat out = cv::Mat(cv::Size(Config::inst()->im_width, Config::inst()->im_height), MAT_FORMAT_C3);

  int type = (int)getSignal("type");
  if (type == 0) {
    // uniform
    cv::randu(out, cv::Scalar(0,0,0,0), cv::Scalar(255,255,255,255)); 
  } else {
    int mean = getSignal("mean");
    int stddev = getSignal("stddev");
    // TBD handle alpha channel
    cv::randn(out, cv::Scalar(mean,mean,mean,mean), cv::Scalar(stddev,stddev,stddev,stddev)); 

  }
  setImage("out", out);
}


} // namespace bm

