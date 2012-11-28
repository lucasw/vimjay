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


#include "structure.h"

#include <boost/lexical_cast.hpp>

#include <glog/logging.h>

#include "config.h"

// filter Node objects
namespace bm {

Contour::Contour(const std::string name) : ImageNode(name)
{
  cv::Mat tmp;
  setImage("in", tmp);
  //setSignal("mode",0);
}

bool Contour::update()
{
  //if (!ImageNode::update()) return false;
  if (!Node::update()) return false;

  cv::Mat in = getImage("in");
  if (in.empty()) {
    VLOG(2) << name << " in is empty";
    return false;
  }
 
  std::vector<std::vector<cv::Point> > contours0;
  std::vector<cv::Vec4i> hierarchy;
  
  cv::Mat in8;
  cv::cvtColor(in, in8, CV_RGB2GRAY);
  cv::findContours(in8, contours0, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat out = cv::Mat(cv::Size(Config::inst()->im_width, Config::inst()->im_height), MAT_FORMAT_C3, cv::Scalar(0,0,0,255));

  cv::drawContours( out, contours0, -1, cv::Scalar(255,255,255,255));
                //1, CV_AA, hierarchy, std::abs(_levels) );

  setImage("out", out);

  return true;
}

} // namespace bm

