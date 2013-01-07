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
  setSignal("epsilon" , 3.0);

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
 
  if (!isDirty(this, 22)) { return true;}
  
  std::vector<std::vector<cv::Point> > contours_orig;
  cv::Mat in8;
  cv::cvtColor(in, in8, CV_RGB2GRAY);
  cv::findContours(in8, contours_orig, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  const float eps = getSignal("epsilon"); // max error of approx
  contours0.resize(contours_orig.size());
  for( size_t k = 0; k < contours_orig.size(); k++ )
    //approxPolyDP(cv::Mat(contours_orig[k]), contours0[k], eps, true);
    approxPolyDP((contours_orig[k]), contours0[k], eps, true);

  cv::Mat out = cv::Mat(Config::inst()->getImSize(), MAT_FORMAT_C3, cv::Scalar(0,0,0,255));

  cv::drawContours( out, contours0, -1, cv::Scalar(255,255,255,255));
                //1, CV_AA, hierarchy, std::abs(_levels) );

  setImage("out", out);

  return true;
}

ContourFlip::ContourFlip(const std::string name) : Contour(name)
{
  cv::Mat tmp;
  setImage("to_flip", tmp); // the image that will be flipped based on the contour found in in
  //setSignal("mode",0);
  setImage("flipped", tmp, true);
  setImage("dist", tmp, true);
  setImage("mapx", tmp, true);
  setImage("mapy", tmp, true);

  setSignal("border", 0, false, ROLL, 0, 4);
  setSignal("mode", 0, false, ROLL, 0, 4);

  initRemaps(base_x, base_y);
    
  off_x = cv::Mat( Config::inst()->getImSize(), CV_32FC1);
  off_y = off_x.clone();
}

// v w is the two points of the line segment, p is the test point
//float minimum_distance(cv::Mat v, cv::Mat w, cv::Mat p) {
float minimum_distance(cv::Point2f v, cv::Point2f w, cv::Point2f p, cv::Point2f& closest) {
  
  // Return minimum distance between line segment vw and point p
  float l2 = cv::norm(w - v); 
  l2 *= l2;
  //const float l2 = cv::norm(cv::Mat(v - w), cv::NORM_L1);  // i.e. |w-v|^2 -  avoid a sqrt
  if (l2 == 0.0) {
    closest = v;
    return cv::norm(p - v);   // v == w case
  }
  // Consider the line extending the segment, parameterized as v + t (w - v).
  // We find projection of point p onto the line. 
  // It falls where t = [(p-v) . (w-v)] / |w-v|^2
  const float t = ((p - v).dot(w - v)) / l2;
  if (t < 0.0) {
    closest = v;
    return cv::norm(p - v);       // Beyond the 'v' end of the segment
  } else if (t > 1.0) {
    closest = w;
    return cv::norm(p - w);  // Beyond the 'w' end of the segment
  }
    
  closest = v + t * (w - v);  // Projection falls on the segment
 
  const float dist = cv::norm(p - closest);
  if (VLOG_IS_ON(3)) {
    LOG_FIRST_N(INFO, 10) << v.x << " " << v.y << ", " 
      << w.x << " " << w.y << ", " 
      << p.x << " " << p.y << ", "
      << closest.x << " " << closest.y << ", "
      << l2 << " " << t << " " <<  dist
      ;
  }

  return dist;
}

bool ContourFlip::update()
{
  //if (!ImageNode::update()) return false;
  if (!Contour::update()) return false;

  // TBD get dirtiness of in to see if flip map needs to be recomputed
  
  if (!isDirty(this, 23)) { return true;}

  cv::Mat to_flip = getImage("to_flip");
  if (to_flip.empty()) {
    VLOG(2) << name << " in is empty";
    return false;
  }
  
  cv::Mat flipped = cv::Mat(to_flip.size(), to_flip.type());
 
  bool valid;
  bool is_dirty;
  cv::Mat in = getImage("in", valid, is_dirty, 51);
  
  if (is_dirty) 
  {
  
  LOG(INFO) << "contour flip updating " << is_dirty;
  cv::Mat dist = cv::Mat(to_flip.size(), to_flip.type());

  const int wd = dist.cols;
  const int ht = dist.rows;

  // This is very slow for dense contours, maybe make
  // scale option that will process the image at a lower resolution 
  // then upscale the off_x,off_y for the remap
  for (int y = 0; y < ht; y++) {
  for (int x = 0; x < wd; x++) {

  float min_dist = 1e9;
  cv::Point2f min_closest;

  int count = 0;
  // TBD just find the nearest contour point for now, don't worry about long segment
  // or the actual normal of the segment - just flip the pixel on the nearest point
  for (int i = 0; i < contours0.size(); i++) {  
  for (int j = 0; j < contours0[i].size(); j++) { 
  
    cv::Point2f v = contours0[i][j]; 
    cv::Point2f w = contours0[i][ (j+1) % contours0[i].size() ]; 
    //const float dx = (contours0[i][j].x - x); 
    //const float dy = (contours0[i][j].y - y);
    //const float cur_dist = fabs(dx) + fabs(dy); 
    //const float cur_dist = sqrt(dx*dx + dy*dy);
    cv::Point2f closest;
    const float cur_dist = minimum_distance( v, w, cv::Point2f(x, y), closest ); 
    if (cur_dist < min_dist) {
      min_dist = cur_dist;
      min_closest = closest;
    }
    count++;
  }}

  if ( (x == 0) && ( y == 0) ) setSignal("count", count);

  // TBD make a reflection effect instead of straight rolling over the edges?
  const int src_x = ((x + (int) ( 2 * (min_closest.x - x) ) ) + wd) % wd; 
  const int src_y = ((y + (int) ( 2 * (min_closest.y - y) ) ) + ht) % ht;
 
  // TBD this could be a map for remap and if the in image doesn't change it will
  // be more efficient
  //flipped.at<cv::Vec4b>(y, x) = to_flip.at<cv::Vec4b>(src_y, src_x);
  off_x.at<float>(y, x) = src_x - x;
  off_y.at<float>(y, x) = src_y - y;
  //LOG_FIRST_N(INFO,20) << src_x << " " << x << ", " << src_y << " " << y;
  dist.at<cv::Vec4b>(y, x) = cv::Scalar::all(min_dist); // % 255);
  }}

  cv::Mat dist_x = base_x + (off_x); //_scaled - offsetx * scalex);
  cv::Mat dist_y = base_y + (off_y); //_scaled - offsety * scaley);

  cv::convertMaps(dist_x, dist_y, dist_xy16, dist_int, CV_16SC2, true);
  
  setImage("dist", dist);

  {
    cv::Mat dist_xy8;
    cv::Mat dist_xy16_temp;
    cv::convertMaps(off_x, off_y, dist_xy16_temp, dist_int, CV_16SC2, true);
    dist_xy16_temp.convertTo(dist_xy8, CV_8UC2, getSignal("map_scale"));
    cv::Mat mapx = cv::Mat( Config::inst()->getImSize(), CV_8UC4, cv::Scalar(0,0,0,0));
    cv::Mat mapy = cv::Mat( Config::inst()->getImSize(), CV_8UC4, cv::Scalar(0,0,0,0));

    int chx[] = {0,0, 0,1, 0,2};
    mixChannels(&dist_xy8, 1, &mapx, 1, chx, 3 );
    int chy[] = {1,0, 1,1, 1,2};
    mixChannels(&dist_xy8, 1, &mapy, 1, chy, 3 );

    setImage("mapx", mapx);
    setImage("mapy", mapy);
  }

  }
  
 
  if (!dist_xy16.empty()) 
    cv::remap(to_flip, flipped, dist_xy16, cv::Mat(), getModeType(), getBorderType());

  setImage("flipped", flipped);

  return true;
}

} // namespace bm

