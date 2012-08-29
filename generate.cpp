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
  setSignal("y1",100);
  setSignal("x2",200);
  setSignal("y2",50);
  setSignal("x3",300);
  setSignal("y3",150);
  setSignal("num", 4);
}

std::string logMat(const cv::Mat& m) 
{
  std::stringstream s;
  s << "{";
  for (int x = 0; x < m.rows; x++) {
    for (int y = 0; y < m.cols; y++) {
      s << " " << m.at<double>(x,y); 
    }
      s << std::endl;
    
    }
  s << "}";
  return s.str();
}

bool Bezier::update()
{
  //if (!ImageNode::update()) return false;
  if (!Node::update()) return false;

  cv::Mat out = cv::Mat(cv::Size(Config::inst()->width, Config::inst()->height), MAT_FORMAT_C3);
  out = cv::Scalar(0,0,0);

  float x0 = getSignal("x0");
  float y0 = getSignal("y0");
  float x1 = getSignal("x1");
  float y1 = getSignal("y1");
  float x2 = getSignal("x2");
  float y2 = getSignal("y2");
  float x3 = getSignal("x3");
  float y3 = getSignal("y3");
  
  int num = getSignal("num");
  if (num < 2) { num = 2;
    setSignal("num", num);
  }

  double coeff_raw[4][4] = {
      { 1, 0, 0, 0},
      {-3, 3, 0, 0},
      { 3,-6, 3, 0},
      { 1, 3,-3, 1}
      };
  cv::Mat coeff = cv::Mat(4, 4, CV_64F, coeff_raw);

  double control_raw[4][2] = {
      {0, 0},
      {x1-x0, y1-y0},
      {x2-x0, y2-y0},
      {x3-x0, y3-y0}
      };
  cv::Mat control = cv::Mat(4, 2, CV_64F, control_raw);

  VLOG(5) << CLTXT << "coeff " << CLNRM << std::endl << logMat(coeff); 
  VLOG(5) << CLTXT <<"control " << CLNRM << std::endl << logMat(control); 

  cv::Point2f old_pt;

  for (int i = 0; i < num; i++) {
    float t = (float)i/(float)(num-1);
    double tee_raw[1][4] = {{ 1.0, t, t*t, t*t*t}};

    cv::Mat tee = cv::Mat(1, 4, CV_64F, tee_raw);

    cv::Mat pos = tee * coeff * control;

    cv::Point new_pt = cv::Point2f(pos.at<double>(0,0) + x0, pos.at<double>(0,1) + y0);
    
    if (i > 0) 
      cv::line(out, old_pt, new_pt, cv::Scalar(255, 255, 255), 2, CV_AA ); 
    old_pt = new_pt;

    VLOG(5) << "pos " << t << " "
      << new_pt.x << " " << new_pt.y 
      << std::endl << logMat(tee) 
      << std::endl << logMat(pos); 
  }

  setImage("out", out);

  return true;
}


} // namespace bm

