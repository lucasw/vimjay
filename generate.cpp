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

extern "C" {
#include "other/DSOnoises/noise1234.h"
#include "other/DSOnoises/simplexnoise1234.h"
#include "other/DSOnoises/sdnoise1234.h"
#include "other/DSOnoises/srdnoise23.h"
}
//#include "other/simplexnoise.h"
//#include "other/simplextextures.h"
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

  cv::Mat out = cv::Mat(Config::inst()->getImSize(), MAT_FORMAT_C3);
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

  cv::Mat out = cv::Mat(Config::inst()->getImSize(), 
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
  
  cv::Mat out = cv::Mat(Config::inst()->getImSize(), MAT_FORMAT_C3);

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


SimplexNoise::SimplexNoise(const std::string name) : ImageNode(name)
{
  //setSignal("octaves", 2, false, SATURATE, 1, 20);
  //setSignal("persist", 0.8);//, false, SATURATE, 0.0, 1.0);
  
  cv::Mat tmp;
  setImage("dx", tmp, true);
  setImage("dy", tmp, true);
  setImage("dz", tmp, true);
  
  setSignal("type", 0, false, ROLL, 0, 8);

  setSignal("scale", 0.02); 
  setSignal("z", 0.0);
  setSignal("t", 0.0);
  setSignal("off_x_nrm", 0.0);
  setSignal("off_y_nrm", 0.0);

  setSignal("scale_v", 127.0);
  setSignal("off_v", 127.0);
}

bool SimplexNoise::update()
{
  if (!Node::update()) return false;

  // if any inputs have changed this will go on to draw
  const bool id1 = isDirty(this,30);
  //LOG(INFO) << id1;
  if (!id1) {
    return true;
  }
 
  cv::Mat out = cv::Mat(Config::inst()->getImSize(), MAT_FORMAT_C3);
  cv::Mat dx_im = out.clone();
  cv::Mat dy_im = out.clone();
  cv::Mat dz_im = out.clone();

  //const float octaves = getSignal("octaves");
  //const float persist = getSignal("persist");
  const float scale = getSignal("scale");
  const float off_x_nrm = getSignal("off_x_nrm");// * out.cols;
  const float off_y_nrm = getSignal("off_y_nrm");// * out.rows;
  const float off_z = getSignal("z");// * out.rows;
  const float t = getSignal("t");// * out.rows;
  const float scale_v = getSignal("scale_v");// * out.rows;
  const float off_v = getSignal("off_v");// * out.rows;

  int mode = getSignal("type");

  for (int i = 0; i < out.rows; i++) {
  for (int j = 0; j < out.cols; j++) {
    
    const float x = j*scale + off_x_nrm;
    const float y = i*scale + off_y_nrm;
    const float z = off_z;
    
    float val, dx, dy, dz;
    
    if (mode == 0) {
      // noise 1 will be a signal
      val = noise2(x, y);
    } else if (mode == 1) {
      val = noise3(x, y, z);
    } else if (mode == 2) {
      val = noise4(x, y, z, t);

    // TBD pnoise (periodic)

    } else if (mode == 2) {
      val = snoise2(x, y); // TBD is this faster than snoise3?  otherwise get rid of it
    } else if (mode == 3) {
      val = snoise3(x, y, z);
    } else if (mode == 4) {
      val = snoise4(x, y, z, t);

    } else if (mode == 5) {
      val = sdnoise2(x, y, &dx, &dy);
    } else if (mode == 6) {
      val = sdnoise3(x, y, z, &dx, &dy, &dz);
      // TBD sdnoise4?

    } else if (mode == 7) {
      val = srdnoise2(x, y, t, &dx, &dy);
    } else if (mode == 8) {
      val = srdnoise3(x, y, z, t, &dx, &dy, &dz);
    } else {
      val = 0;
    }
    // TBD srdnoise4
    
    val = val * scale_v + off_v;
    dx = dx * scale_v + off_v;
    dy = dy * scale_v + off_v;
    dz = dz * scale_v + off_v;

    // TBD rollover can be interesting, but saturate for now
    if (val > 255) val = 255;
    if (val < 0) val = 0;
    //float val = marble_noise_2d(octaves, persist, scale, j + off_x_nrm, i + off_y_nrm)*127+127;
    cv::Vec4b col = cv::Vec4b(val, val, val, 0);
    out.at<cv::Vec4b>(i,j) = col;

    if (mode >= 5) {
      // TBD this is somewhat wasteful, could combine all dx,dy,dz into rgb channels
      col = cv::Vec4b(dx, dx, dx, 0);
      dx_im.at<cv::Vec4b>(i, j) = col;
      
      col = cv::Vec4b(dy, dy, dy, 0);
      dy_im.at<cv::Vec4b>(i, j) = col;

      col = cv::Vec4b(dz, dz, dz, 0);
      dz_im.at<cv::Vec4b>(i, j) = col;
    }

  }}

  setImage("out", out);
  if (mode >= 5) {
    setImage("dx", dx_im);
    setImage("dy", dy_im);
    setImage("dz", dz_im);
  }
}

} // namespace bm

