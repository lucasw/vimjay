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


#include "filter.h"

#include <boost/lexical_cast.hpp>
#include <opencv2/video/tracking.hpp>

#include <glog/logging.h>

#include <opencv2/videostab/videostab.hpp>

// filter Node objects
namespace bm {

FilterFIR::FilterFIR(const std::string name) : Buffer(name)
{

}

void FilterFIR::setup(const std::vector<float> new_xi)
{
  xi = new_xi;
  setSignal("max_size", new_xi.size());
}

bool FilterFIR::update()
{
  bool rv = Node::update();
  if (!rv) return false;
  
  if (!isDirty(this,22)) { return true;}
  
  manualUpdate();

  cv::Mat out;

  int addnum =  0;
  for (int i = 0; i < ports.size(); i++) {
    if (ports[i]->type != SIGNAL) continue;
    const std::string port = ports[i]->name;

    if (port.substr(0,2) != "xi") continue;
    float val = getSignal(port);

    cv::Mat tmp = frames[frames.size() - addnum - 1];  

    if (addnum == 0) 
      out = tmp * val;
    else {
      if (val > 0) {
        out += tmp * val;
      } else 
        out -= tmp * -val;
    }
    
    addnum++;
    
    if (addnum >= frames.size()) break;
  }

  setImage("out", out);

  return true;
}

bool FilterFIR::handleKey(int key)
{
  bool valid_key = Buffer::handleKey(key);
  if (valid_key) return true;

  valid_key = true;
  if (key == '[') {

    // add an input addition port, TBD move to function
    int add_num = 0;
    for (int i = 0; i < ports.size(); i++) {
      if (ports[i]->type != SIGNAL) continue;
      const std::string port = ports[i]->name;

      if (port.substr(0,2) != "xi") {
        VLOG(1) << name << " : " << port.substr(0,2) << " " << port;
        continue;
      }
      add_num++;
    }

    setSignal("max_size", add_num+1);

    // add a new addition port
    const std::string port = "xi" + boost::lexical_cast<std::string>(add_num);
    setInputPort(SIGNAL, port, NULL, "value"); // this allows other signals to connect to replace nf

    // TBD make a way to delete a port
  } else {
    valid_key = false;
  }

  // TBD 
  if (valid_key) setDirty();

  return valid_key;
}


Sobel::Sobel(const std::string name) : ImageNode(name)
{
  cv::Mat tmp;
  setImage("in", tmp);
  setSignal("xorder",1);
  setSignal("yorder",0);
  setSignal("ksize",3);
  setSignal("scale",0.8);
  setSignal("delta",128);
}

bool Sobel::update()
{
  //if (!ImageNode::update()) return false;
  if (!Node::update()) return false;

  cv::Mat in = getImage("in");
  if (in.empty()) {
    VLOG(2) << name << " in is empty";
    return false;
  }

  int ksize = getSignal("ksize");
  int xorder = getSignal("xorder"); 
  int yorder = getSignal("yorder"); 
  float scale = getSignal("scale");

  if (ksize < 1) ksize = 1;
  if (ksize > 4) ksize = 4;
  if (xorder < 0) xorder = 0;
  if (xorder > 4) xorder = 4;
  if (yorder < 0) yorder = 0;
  if (yorder > 4) yorder = 4;
  if (scale < 0.001) scale = 0.001;

  if (xorder + yorder < 1) xorder = 1;
  
  setSignal("ksize", ksize);
  setSignal("xorder", xorder);
  setSignal("yorder", yorder);
  setSignal("scale", scale);

  int real_ksize = 1;
  if (ksize == 2) real_ksize = 3;
  if (ksize == 3) real_ksize = 5;
  if (ksize == 4) real_ksize = 7;

  cv::Mat out;
  cv::Sobel(in, out, in.depth(), 
      xorder,
      yorder,
      real_ksize, 
      scale,
      getSignal("delta")
      );
  setImage("out", out);

  return true;
}

Laplacian::Laplacian(const std::string name) : ImageNode(name)
{
  cv::Mat tmp;
  setImage("in", tmp);
  // TBD Signals should allo min max parameters to be set
  setSignal("ksize", 1);
  // scale and offset
  setSignal("scale",1.0);
  setSignal("delta",128);
}

bool Laplacian::update()
{
  const bool rv = Node::update();
  if (!rv) return false;

  if (!isDirty(this, 5)) { 
    VLOG(1) << name << " not dirty ";
    return true; 
  }

  cv::Mat in = getImage("in");
  if (in.empty()) return true;

  int ksize = getSignal("ksize");

  if (ksize < 0) { ksize = 0; setSignal("ksize", ksize); }
  if (ksize > 15) { ksize = 15; setSignal("ksize", ksize); }
  int real_ksize = ksize*2 + 1;

  cv::Mat out;
  cv::Laplacian(in, out, in.depth(),
      real_ksize,
      getSignal("scale"), 
      getSignal("delta"), 
      getBorderType(true)
      );

  setImage("out", out);

  return true;
}

////////////////////////////////////////
  GaussianBlur::GaussianBlur(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("k_width",2);
    setSignal("k_height",2);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool GaussianBlur::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
    
    cv::Mat in = getImage("in");
    if (in.empty()) return false;

    int k_width = abs(getSignal("k_width"));
    int k_height = abs(getSignal("k_height"));
     
    cv::Size ksize = cv::Size(k_width*2 + 1, k_height*2+1);

    cv::Mat out = cv::Mat(in.size(), in.type());

    cv::GaussianBlur(in, out, ksize, 0, 0, cv::BORDER_REFLECT);
    
    setImage("out", out);
    return true;
  }

////////////////////////////////////////
  MedianBlur::MedianBlur(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("k_size", 2, false, SATURATE, 1, 10);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool MedianBlur::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
    
    cv::Mat in = getImage("in");
    if (in.empty()) return false;

    int k = getSignal("k_size");
     
    int ksize = k * 2 + 1;

    cv::Mat out = cv::Mat(in.size(), in.type());

    cv::medianBlur(in, out, ksize);
    
    setImage("out", out);
    return true;
  }

  BilateralFilter::BilateralFilter(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("d", 2, false, SATURATE, -1, 10);
    setSignal("sigma_color", 10);
    setSignal("sigma_space", 10);
    setSignal("border", 0, false, ROLL, 0, 4);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool BilateralFilter::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
    
    cv::Mat in = getImage("in");
    if (in.empty()) return false;

    cv::Mat in_3 = cv::Mat(in.size(), CV_8UC3, cv::Scalar(0));  
    // just calling reshape(4) doesn't do the channel reassignment like this does
    int ch[] = {0,0, 1,1, 2,2};
    cv::mixChannels(&in, 1, &in_3, 1, ch, 3 );

    const float sigma_color = getSignal("sigma_color");
    const float sigma_space = getSignal("sigma_space");
    const int d = getSignal("d");
    
    cv::Mat out_3;

    cv::bilateralFilter(in_3, out_3, d, sigma_color, sigma_space, getBorderType());
    
    cv::Mat out = cv::Mat(out_3.size(), CV_8UC4, cv::Scalar(0));  
    cv::mixChannels(&out_3, 1, &out, 1, ch, 3 );

    setImage("out", out);

    return true;
  }

InPaint::InPaint(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setImage("mask", tmp);
    setSignal("radius", 10);
    setSignal("mode", 0, false, ROLL, 0, 1);
  }

  bool InPaint::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
    
    cv::Mat in = getImage("in");
    if (in.empty()) return false;

    cv::Mat in_3 = cv::Mat(in.size(), CV_8UC3, cv::Scalar(0));  
    // just calling reshape(4) doesn't do the channel reassignment like this does
    int ch[] = {0,0, 1,1, 2,2};
    cv::mixChannels(&in, 1, &in_3, 1, ch, 3 );

    cv::Mat mask = getImage("mask");
    if (mask.empty()) return false;

    cv::Mat mask_1 = cv::Mat(in.size(), CV_8UC1, cv::Scalar(0));  
    int ch1[] = {0, 0};
    mixChannels(&mask, 1, &mask_1, 1, ch1, 1);
  
    int mode_ind = getSignal("mode");
    int mode = cv::INPAINT_NS;
    
    if (mode_ind == 1) {
      mode = cv::INPAINT_TELEA;
    }

    cv::Mat out_3;
    cv::inpaint(in_3, mask_1, out_3, getSignal("radius"), mode);

    {
      cv::Mat out = cv::Mat(out.size(), CV_8UC4, cv::Scalar(0,0,0,0));
      int ch[] = {0,0, 1,1, 2,2}; 
      mixChannels(&out_3, 1, &out, 1, ch, 3);
      setImage("out", out);
    }
    return true;

}

///////////////////////////////////////////////////////////////////////////// 
MorphologyEx::MorphologyEx(const std::string name) : ImageNode(name)
{
  cv::Mat in;
  setImage("in", in);
  setSignal("element", 0, false, ROLL, 0, 2);
  setSignal("element_size_x", 3, false, SATURATE, 1, 10);
  setSignal("element_size_y", 3, false, SATURATE, 1, 10);
  setSignal("op", 0, false, ROLL, 0, 6);
  setSignal("iterations", 1, false, SATURATE, 1, 10);

  // TBD could allow element image input, which would be shrunk
  // down to element size to be used as the morph element
}

bool MorphologyEx::update() 
{
  if (!Node::update()) return false;

  if (!isDirty(this, 5)) { 
    VLOG(1) << name << " not dirty ";
    return true; 
  }

  cv::Mat in = getImage("in");
  if (in.empty()) return false;
 

  int element_shape_ind = getSignal("element");
  int element_shape = cv::MORPH_RECT;
  if (element_shape_ind == 1) element_shape = cv::MORPH_CROSS;
  if (element_shape_ind == 2) element_shape = cv::MORPH_ELLIPSE;

  int element_size_x = getSignal("element_size_x");
  int element_size_y = getSignal("element_size_y");
  int op_ind = getSignal("op");

  // TBD should the default do nothing?
  int op = cv::MORPH_OPEN;
  if (op_ind == 1) op = cv::MORPH_CLOSE;
  else if (op_ind == 2) op = cv::MORPH_GRADIENT;
  else if (op_ind == 3) op = cv::MORPH_BLACKHAT;
  else if (op_ind == 4) op = cv::MORPH_TOPHAT;
  else if (op_ind == 5) op = cv::MORPH_DILATE;
  else if (op_ind == 6) op = cv::MORPH_ERODE;

  int iterations = getSignal("iterations");

  cv::Mat kernel = cv::getStructuringElement(
      element_shape, 
      cv::Size(element_size_x*2+1, element_size_y*2+1), 
      cv::Point(element_size_x, element_size_y) );

  cv::Mat out;
  
  //if (op_ind <= 6) 
  {
  cv::morphologyEx(in, out, 
      op,
      kernel,  
      cv::Point(-1, -1),
      iterations,
      getBorderType()
      );
  } 
  #if 0
  // these are identical to passing in MORPH_DILATE/ERODE
  else if (op_ind == 7) {
    cv::dilate(in, out, kernel, cv::Point(-1,-1), iterations, getBorderType()); 
  } else if (op_ind == 8) {
    cv::erode(in, out, kernel, cv::Point(-1,-1), iterations, getBorderType()); 
  }
  #endif
  setImage("out", out);
}

  ///////////////////////////////////////////////////////////////////////////// 
  OpticalFlow::OpticalFlow(const std::string name) : Remap(name)
  {
    cv::Mat tmp, tmp2, tmp3, tmp4;
    //setImage("prev", tmp);
    setImage("next", tmp2);
    //setImage("flowx", tmp3);
    //setImage("flowy", tmp4);
    setSignal("pyr_scale",0.5);
    setSignal("levels",1);
    setSignal("winsize",16);
    setSignal("iterations",2);
    setSignal("poly_n",5);
    setSignal("poly_sigma",1.1);
    setSignal("mode",0, false, ROLL, 0, 3);
    
    setSignal("scale",1.0);
    setSignal("offset",128);

    setSignal("interp", 0.5);
    setSignal("interp_mode",0, false, ROLL, 0, 2);
  }

  bool OpticalFlow::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(5) << name << " not dirty ";
      return true; 
    }
 
    float pyr_scale = getSignal("pyr_scale");
    // TBD add min/max and type to setSignal()
    if (pyr_scale < 0.1) pyr_scale = 0.1;
    if (pyr_scale > 0.9) pyr_scale = 0.9;
    setSignal("pyr_scale", pyr_scale);

    int levels = getSignal("levels");
    if (levels < 1) levels = 1;
    setSignal("levels", levels);

    int winsize = getSignal("winsize");
    if (winsize < 4) winsize = 4;
    setSignal("winsize", winsize);

    int iterations = getSignal("iterations");
    if (iterations < 1) iterations = 1;
    setSignal("iterations", iterations);

    int poly_n = getSignal("poly_n");
    if (poly_n < 2) poly_n = 2;
    setSignal("poly_n", poly_n);

    float poly_sigma = getSignal("poly_sigma");

    int mode = getSignal("mode");

    int flow_mode = 0;
    if (mode == 1) flow_mode = cv::OPTFLOW_USE_INITIAL_FLOW;
    if (mode == 2) flow_mode = cv::OPTFLOW_USE_INITIAL_FLOW & cv::OPTFLOW_FARNEBACK_GAUSSIAN;
    if (mode == 3) flow_mode = cv::OPTFLOW_FARNEBACK_GAUSSIAN;


    // clear previous setSignal dirtiness
    isDirty(this, 5);

    cv::Mat prev = getImage("in");
    cv::Mat next = getImage("next");
    if (prev.empty() || next.empty()) return true;


    cv::Mat prevm, nextm;

    cv::cvtColor(prev, prevm, CV_BGR2GRAY);
    cv::cvtColor(next, nextm, CV_BGR2GRAY);

    cv::calcOpticalFlowFarneback(prevm, nextm, flow, 
        pyr_scale, levels, winsize, 
        iterations, poly_n, poly_sigma, flow_mode);
    
    cv::Mat flow8_2;
    flow.convertTo(flow8_2, CV_8UC2, getSignal("scale"), getSignal("offset"));
    // TBD use scalex and scaley to separately scale those? 
    {
      cv::Mat offx = cv::Mat(flow8_2.size(), CV_8UC4, cv::Scalar(0));
      int ch[] = {0,0, 0,1, 0,2}; 
      mixChannels(&flow8_2, 1, &offx, 1, ch, 3);
      setImage("offx", offx);
    }
    {
      cv::Mat offy = cv::Mat(flow8_2.size(), CV_8UC4, cv::Scalar(0));
      int ch[] = {1,0, 1,1, 1,2}; 
      mixChannels(&flow8_2, 1, &offy, 1, ch, 3);
      setImage("offy", offy);
    }   
   
    /// Do interpolation, should make optional
    {
      const float interp = getSignal("interp");
      const int interp_mode = getSignal("interp_mode");
    
      cv::Mat out_forward, out_reverse, out;

      /*
        oflow
        prev(x,y) ~= next(x + flow_x(x,y), y + flow_y(x,y))
        remap
i       dst(x,y) = src(map_x(x,y), map_y(x,y))
      */

      if ((interp_mode == 0) || (interp_mode == 2)) {
        cv::remap(prev, out_forward, base_xy - flow*interp, cv::Mat(), cv::INTER_NEAREST, cv::BORDER_REPLICATE);
      }
      if ((interp_mode == 1) || (interp_mode == 2)) {

        //cv::Mat flow_reverse;
        cv::calcOpticalFlowFarneback(nextm, prevm, flow_reverse, 
          pyr_scale, levels, winsize, 
          iterations, poly_n, poly_sigma, flow_mode);
        cv::remap(next, out_reverse, base_xy - flow_reverse*(1.0 - interp), cv::Mat(), cv::INTER_NEAREST, cv::BORDER_REPLICATE);
      }

      cv::Mat test;
      if (interp_mode == 0) {
        out = out_forward;
        test = prev.clone();
      }
      else if (interp_mode == 1) {
        out = out_reverse;
        test = next.clone();
      }
      else if (interp_mode == 2) {
        out = out_forward * (1.0 - interp) + out_reverse * interp;
        test = prev/2 + next/2;
      }

      setImage("out", out);

      // TBD temp/optional
      // make test image
      for (int i = 0; i < flow.rows; i += 14) {
      for (int j = 0; j < flow.cols; j += 14) {
        float dx = flow.at<cv::Point2f>(i,j).x;
        float dy = flow.at<cv::Point2f>(i,j).y;
        if ((dx > 2.0) || (dy > 2.0)) {
        cv::Point2f p1 = cv::Point2f(j, i);
        cv::Point2f p2 = cv::Point2f((float)j + dx, (float)i + dy); 
        cv::line(test, p1, p2, cv::Scalar(0, 0, 0, 0), 1);
        cv::circle(test, p1, 2, cv::Scalar(0,0,255,0));
        cv::circle(test, p2, 2, cv::Scalar(0,255,0,0));
        }
      }}
      setImage("test", test);
    }
  
    return true;
  }
} // namespace bm

