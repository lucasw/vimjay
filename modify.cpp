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

#include "modify.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>

#include <deque>

#include "config.h"

using namespace cv;
using namespace std;

namespace bm {
//////////////////////////////////////////////////

  cv::Mat chan3to4(const cv::Mat& in_3)
  {
    cv::Mat out = cv::Mat( in_3.size(), CV_8UC4, cv::Scalar::all(0) );
    int ch3[] = {0,0, 1,1, 2,2}; 
    mixChannels(&in_3, 1, &out, 1, ch3, 3 );
    return out;
  } 

  cv::Mat chan4to3(const cv::Mat& in_4)
  {
    cv::Mat out = cv::Mat( in_4.size(), CV_8UC3, cv::Scalar::all(0) );
    int ch3[] = {0,0, 1,1, 2,2}; 
    mixChannels(&in_4, 1, &out, 1, ch3, 3 );
    return out;
  } 

  cv::Mat chan1to4(const cv::Mat& in_1)
  {
    cv::Mat out_3;
    cv::cvtColor(in_1, out_3, CV_GRAY2BGR);
    return chan3to4(out_3);
  }

  cv::Mat chan4to1(const cv::Mat& in_4)
  {
    cv::Mat out = cv::Mat( in_4.size(), CV_8UC1);
    // TBD use BGR2GRAY?
    int ch1[] = {0, 0};
    mixChannels(&in_4, 1, &out, 1, ch1, 1);
    return out;
  }

  Rot2D::Rot2D(const std::string name) : ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in",tmp);
    setSignal("phi", 0);
    setSignal("theta", 0);
    setSignal("psi", 0);
    setSignal("z", 573, false, SATURATE, 1, 1e6); // TBD this number
    setSignal("scale", 1.0);
    setSignal("center_x", 5); //  Config::inst()->im_width/2 );
    setSignal("center_y", 5); // Config::inst()->im_height/2 );
    setSignal("center_z", 0); // -Config::inst()->im_height/2 );
    setSignal("off_x", 5); //Config::inst()->im_width/2 );
    setSignal("off_y", 5); //Config::inst()->im_height/2 );
    setSignal("off_z", 0);
    setSignal("nrm_px", 1, false, ROLL, 0, 1); // normalized (0-10) or pixel coordinates for above
    setSignal("border", 0, false, ROLL, 0, 4);
    setSignal("mode", 0, false, ROLL, 0, 4);
    setSignal("manual_xy", 0.0);

    setSignal("x0", 0 );
    setSignal("x1", 0 );
    setSignal("x2", 0 );
    setSignal("x3", 0 );
  }

  bool Rot2D::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this,40)) return true;

    bool im_dirty;
    cv::Mat in;
    in = getImage("in");
    if (in.empty()) return false;

    const float wd = Config::inst()->im_width;
    const float ht = Config::inst()->im_height;

    // TBD provide normalized, radians, and angle input select
    // euler angles
    float phi   = getSignal("phi")   * M_PI/180.0;    
    float theta = getSignal("theta") * M_PI/180.0;    
    float psi   = getSignal("psi")   * M_PI/180.0;   

    float scale = getSignal("scale");    

    const bool nrm_px = getSignal("nrm_px");
    
    cv::Point3f center;
    center.x = getSignal("center_x");     
    center.y = getSignal("center_y");
    center.z = getSignal("center_z");
  
    float off_x = getSignal("off_x");     
    float off_y = getSignal("off_y");
    float off_z = getSignal("off_z");

    if (nrm_px) {
      center.x *= wd/(10.0);
      center.y *= ht/(10.0);
      center.z *= ht/(10.0);
      
      off_x *= wd/(10.0);
      off_y *= ht/(10.0);
      off_z *= ht/(10.0);
    }
        //VLOG(1) << name << " " << is_dirty << " " << im_in->name << " " << im_in->is_dirty;


    cv::Mat in_p = (cv::Mat_<float>(3,4) << 
        0, wd, wd, 0, 
        0, 0,  ht, ht,
        0, 0, 0, 0);

    cv::Mat in_roi = in_p.t()(cv::Rect(0, 0, 2, 4)); //).clone();
    in_roi = in_roi.clone(); 
    cv::Mat out_roi = in_roi.clone();
 
    if (getSignal("manual_xy") < 0.5) {
      //////////////////////////////////////////////////
      /// This implements a standard rotozoom
      // move the image prior to rotation
      cv::Mat offset = (cv::Mat_<float>(3,4) << 
          off_x, off_x, off_x, off_x, 
          off_y, off_y, off_y, off_y,
          off_z, off_z, off_z, off_z);

      // shift the image after rotation
      cv::Mat center_m = (cv::Mat_<float>(3,4) << 
          center.x, center.x, center.x, center.x, 
          center.y, center.y, center.y, center.y,
          center.z, center.z, center.z, center.z);

      // Rotation matrices
      cv::Mat rotz = (cv::Mat_<float>(3, 3) <<
          cos(phi), -sin(phi), 0, 
          sin(phi),  cos(phi), 0,
          0, 0, 1);

      cv::Mat roty = (cv::Mat_<float>(3, 3) <<
          cos(theta),  0, sin(theta),  
          0, 1, 0,
          -sin(theta), 0, cos(theta) );

      cv::Mat rotx = (cv::Mat_<float>(3, 3) <<
          1,  0,        0,
          0,  cos(psi), sin(psi),  
          0, -sin(psi), cos(psi) );

      // TBD reformat the matrices so all the transposes aren't necessary

      // Transform into ideal coords
      //float fx = getSignal("fx");
      cv::Mat out_p = (in_p - offset).t() * rotx.t() * roty.t() * rotz.t() * scale; // + center_m.t();
    
      out_roi = out_p(cv::Rect(0, 0, 2, 4)).clone();
     
      // this moves the image away from the 0 plane
      const float z = getSignal("z");
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
          out_roi.at<float>(i,j) = z * out_p.at<float>(i,j) / (out_p.at<float>(i,2) + z) + center_m.at<float>(j,i);
        }
      }

      //
      setSignal("x0", out_roi.at<float>(0,0));
      setSignal("y0", out_roi.at<float>(0,1));
      setSignal("x1", out_roi.at<float>(1,0));
      setSignal("y1", out_roi.at<float>(1,1));
      setSignal("x2", out_roi.at<float>(2,0));
      setSignal("y2", out_roi.at<float>(2,1));
      setSignal("x3", out_roi.at<float>(3,0));
      setSignal("y3", out_roi.at<float>(3,1));
      //
      setSignal("z0", out_p.at<float>(0,2));
      setSignal("z1", out_p.at<float>(1,2));
      setSignal("z2", out_p.at<float>(2,2));
      setSignal("z3", out_p.at<float>(3,2));
    } else {   
      out_roi.at<float>(0,0) = getSignal("x0");
      out_roi.at<float>(1,0) = getSignal("x1");
      out_roi.at<float>(2,0) = getSignal("x2");
      out_roi.at<float>(3,0) = getSignal("x3");
      out_roi.at<float>(0,1) = getSignal("y0");
      out_roi.at<float>(1,1) = getSignal("y1"); 
      out_roi.at<float>(2,1) = getSignal("y2");
      out_roi.at<float>(3,1) = getSignal("y3"); 
    }

    //  cv::Mat out_p_2d = cv::Mat_<float>(4, 2);
    //  out_p_2d.at<float>(i,j) = ;

    cv::Mat out;
    // TBD make inter_nearest changeable
    cv::Mat transform = cv::getPerspectiveTransform(in_roi, out_roi);
    cv::warpPerspective(in, out, transform, 
        in.size(), getModeType(), getBorderType());
    setImage("out", out);

    #if 0
    for (int i = 0; i < transform.rows; i++) {
    for (int j = 0; j < transform.cols; j++) {
      std::string name = "m" + boost::lexical_cast<string>(i) + boost::lexical_cast<string>(j);
      setSignal(name, out_p.at<float>(i,j));
    }}
    #endif

    return true;
  }

  //
  Kaleid::Kaleid(const std::string name) : ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in",tmp);

    setImage("mask",tmp);
  
    setImage("mapx", tmp, true);
    setImage("mapy", tmp, true);
    setSignal("map_scale", 255.0/(float)Config::inst()->getImSize().width);

    // these govern the tiling of the masked input image
    setSignal("x_off", 100);  
    setSignal("y_off", 100); 
    // tbd need another x,y offset to govern vertical tiling?
    
    setSignal("border", 0, false, ROLL, 0, 4);
    setSignal("mode", 0, false, ROLL, 0, 4);
    // TBD mode selector to toggle how pieces are tiled - flip u/d, l/r, 
    // TBD tiling pieces can be rotated

    initRemaps(base_x, base_y);

    #if 0
    setSignal("x0", 0); setSignal("y0", 0);
    setSignal("x1", 100); setSignal("y1", 0);
    setSignal("x2", 100); setSignal("y2", 100);
    #endif
  }

  bool Kaleid::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this,40)) return true;

    bool im_dirty;
    cv::Mat in;
    in = getImage("in");
    if (in.empty()) return false;

    const float wd = Config::inst()->im_width;
    const float ht = Config::inst()->im_height;

    // multiply mask by base_x and base_y
    // then start at -wd, -ht and start using masked addition to tile
    // the masked image over a new base_x,y that will be given to the remap
    // that concludes the function

    cv::Mat mask4 = getImage("mask");
    // derive the mask from the second input
    if (mask4.empty()) {
      // probably want to change add1 to black and white and then use it here,
      // otherwise color channels get masked individually. (a pixel that is 0 in red
      // but not in green will have different masks on those channels)
      mask4 = in.clone();
    }
    cv::Mat mask1 = chan4to1(mask4); 
        //const int offset = getSignal("offset");
    mask1 = (mask1 > getSignal("offset"));
    
    cv::Mat mask;
    mask1.convertTo(mask, CV_32FC1, 1.0/255.0);
    
    // TBD could hold on to the base_x/y_mask and only update when the mask or the offsets
    // change
    cv::Mat base_x_masked = base_x.mul(mask);
    cv::Mat base_y_masked = base_y.mul(mask);

    cv::Mat total_base_x = cv::Mat( Config::inst()->getImSize(), CV_32FC1, cv::Scalar(0));
    cv::Mat total_base_y = total_base_x.clone(); 
    
    const float x_off = getSignal("x_off");
    const float y_off = getSignal("y_off");
    const int j_max = getSignal("y_repeat");
    const int i_max = getSignal("x_repeat");

    const float theta = getSignal("theta") * M_PI/180.0;

    const float x1o =  0;// - wd/2;
    const float x2o =  wd;//+ wd/2;
    const float y1o =  0;//- ht/2;
    const float y2o =  ht;//+ ht/2;
    cv::Mat in_pts = (cv::Mat_<float>(4,2) <<
        x1o, y1o,
        x1o, y2o,
        x2o, y2o,
        x2o, y1o
        );
    cv::Mat offset_pts = (cv::Mat_<float>(4,2) <<
        wd/2, ht/2,
        wd/2, ht/2,
        wd/2, ht/2,
        wd/2, ht/2
        );

    for (int j = -j_max; j <= j_max; j++) {
    for (int i = -i_max; i <= i_max; i++) {
      // now start shifting
 
      cv::Mat rot = (cv::Mat_<float>(2, 2) <<
        cos(theta*i), -sin(theta*i),  
        sin(theta*i),  cos(theta*i) 
        );   
      cv::Mat out_pts = (in_pts - offset_pts) * rot + offset_pts;
      
      for (int k = 0; k < 4; k++)
        out_pts.at<float>(k,0) += x_off*i;// + wd/2;
      
      for (int k = 0; k < 4; k++)
        out_pts.at<float>(k,1) += y_off*j;// + ht/2;

      if (getSignal("do_y_offset") > 0.5) { 
      }

      if (getSignal("do_flip") > 0.5) {
      }
   

      // need four points
      cv::Mat transform = getPerspectiveTransform(in_pts, out_pts);
    
      cv::Mat x_tf, y_tf;
      // TBD combine using base_xy
      cv::warpPerspective(base_x_masked, x_tf, transform, base_x_masked.size()); //, getModeType(), getBorderType());
      cv::warpPerspective(base_y_masked, y_tf, transform, base_y_masked.size()); //, getModeType(), getBorderType());
      //cv::Mat test= x_tf ==0; // these comparison produce 8 bit images
      // can't do simple binary comparisons because they produce 8-bit images 
      cv::Mat x_tf_neg;
      cv::threshold(x_tf, x_tf_neg, 0.1, 1.0, THRESH_BINARY_INV);
      total_base_x = total_base_x.mul(x_tf_neg);
      total_base_x += x_tf; 
      
      cv::Mat y_tf_neg;
      cv::threshold(y_tf, y_tf_neg, 0.1, 1.0, THRESH_BINARY_INV);
      total_base_y = total_base_y.mul(y_tf_neg);
      total_base_y += y_tf; 
    }}

    cv::Mat dist_xy16, dist_int;
    cv::convertMaps(total_base_x, total_base_y, dist_xy16, dist_int, CV_16SC2, true);
    cv::Mat out;
   
    {
    cv::Mat dist_xy8;
    dist_xy16.convertTo(dist_xy8, CV_8UC2, getSignal("map_scale"));
    cv::Mat mapx = cv::Mat( Config::inst()->getImSize(), CV_8UC4, cv::Scalar(0,0,0,0));
    cv::Mat mapy = cv::Mat( Config::inst()->getImSize(), CV_8UC4, cv::Scalar(0,0,0,0));

    int chx[] = {0,0, 0,1, 0,2}; 
    mixChannels(&dist_xy8, 1, &mapx, 1, chx, 3 );
    int chy[] = {1,0, 1,1, 1,2}; 
    mixChannels(&dist_xy8, 1, &mapy, 1, chy, 3 );
        

    setImage("mapx", mapx);
    setImage("mapy", mapy);
    }
    ///////////////////////
    
    cv::remap(in, out, dist_xy16, cv::Mat(), getModeType(), getBorderType());

   #if 0
    cv::Mat in_pts = (cv::Mat_<float>(2,3) <<
      getSignal("x0"), getSignal("x1"), getSignal("x2"),
      getSignal("y0"), getSignal("y1"), getSignal("y2") );

 
    // specify three points that will be the destination points
    // an isosceles triange with 60 angle

    float l = wd;
    if (wd > ht) {
      l = ht;
    } 

    cv::Mat out_pts = (cv::Mat_<float>(2,3) <<
        0, l, l,
        ht/2, ht/2-l/2, ht/2+l/2 
        );
  
    cv::Mat transform = cv::getPerspectiveTransform(in_pts, out_pts);

    // Create a mask that blanks out parts of the image outside the corner

    // warp the image to be an isosceles triangle with an angle of 60 degrees
      
    // rotate and tile the image in the output image
    #endif


    setImage("out", out);

    return true;
  }
  //
  Undistort::Undistort(const std::string name) : ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in",tmp);
    setSignal("fx", Config::inst()->im_width/2 );
    setSignal("fy", Config::inst()->im_width/2 );
    setSignal("cx", Config::inst()->im_width/2 );
    setSignal("cy", Config::inst()->im_height/2 );
    setSignal("d0", 0 );
    setSignal("d1", 0 );
    setSignal("d2", 0 );
    setSignal("d3", 0 );
    
    setSignal("d4", 0 );
    setSignal("d5", 0 );
    setSignal("d6", 0 );
    setSignal("d7", 0 );
  }

  bool Undistort::update()
  {
    if (!Node::update()) return false;
    
    if (!isDirty(this,40)) return true;

    cv::Mat in = getImage("in");
    if (in.empty()) return false;

    cv::Mat intrinsics = (cv::Mat_<float>(3,3) <<
        getSignal("fx"), 0, getSignal("cx"),
        0, getSignal("fy"), getSignal("cy"),
        0, 0, 1
        );
    // has to be size 4, 5, or 8
    // TBD with 1/100 scale factor
    cv::Mat dist = (cv::Mat_<float>(8,1) <<
        getSignal("d0")/100.0,
        getSignal("d1")/100.0,
        getSignal("d2")/100.0,
        getSignal("d3")/100.0,
        getSignal("d4")/100.0,
        getSignal("d5")/100.0,
        getSignal("d6")/100.0,
        getSignal("d7")/1000.0
        );

    cv::Mat out;

    cv::undistort(in, out, intrinsics, dist);
    setImage("out",out);
    return true;
  }

  Remap::Remap(const std::string name) :
      ImageNode(name)
  {
    cv::Mat in;
    setImage("in", in);
    cv::Mat offx;
    setImage("offx", offx);
    setSignal("scalex", 1.0);
    setSignal("offsetx", 127.0);
    cv::Mat offy;
    setImage("offy", offy);
    setSignal("scaley", 1.0);
    setSignal("offsety", 127.0);
    // select if base indices are added to offxy or not TBD current bit depth doesn't support
    // this
    //setSignal("off_mode", 0, false, ROLL, 0, 1);
    setSignal("border", 0, false, ROLL, 0, 4);
    setSignal("mode", 0, false, ROLL, 0, 4);


    base_x = cv::Mat( Config::inst()->getImSize(),
      CV_32FC1);
    base_y = base_x.clone();
    base_xy = cv::Mat( Config::inst()->getImSize(),
      CV_32FC2);

    for (int i = 0; i < base_x.rows; i++) {
    for (int j = 0; j < base_x.cols; j++) {
      base_x.at<float>(i,j) = j;
      base_y.at<float>(i,j) = i;
    }
    }

    int ch1[] = {0, 0};
    mixChannels(&base_x, 1, &base_xy, 1, ch1, 1);
    int ch2[] = {0, 1};
    mixChannels(&base_y, 1, &base_xy, 1, ch2, 1);

  } 

  bool Remap::update() 
  {
    if (!Node::update()) return false;
    
    if (!isDirty(this,40)) return true;

    cv::Mat in = getImage("in");
    if (in.empty()) return false;

    cv::Mat offx = getImage("offx");
    cv::Mat offy = getImage("offy");

    cv::Mat offx_scaled;
    cv::Mat offy_scaled;

    if (offy.empty() && offx.empty()) {
    //if (offy.empty() || offx.empty()) {
      setImage("out", in);
      return false;
    }
  
    // TBD changing resolution will always cause these to scale differently, may want to normalize
    // with image dimensions so scalex = 100 or 10 is always displacment the width of the image
    float scalex = getSignal("scalex");
    float scaley = getSignal("scaley");
    
    const float offsetx = getSignal("offsetx");
    const float offsety = getSignal("offsety");
    if (offx.empty() ) {
      offx = cv::Mat(offy.size(), CV_8UC4, cv::Scalar(0));
      scalex = 0;
    }
    if (offy.empty() ) {
      offy = cv::Mat(offx.size(), CV_8UC4, cv::Scalar(0));
      scaley = 0;
    }
    /* else if (offy.empty() && !offx.empty()) {
      offy_scaled = cv::Mat(base_x.size(), CV_16SC1, cv::Scalar(0));
      offx.convertTo(offx_scaled, CV_16SC1, getSignal("scalex"));
    } else if (!offy.empty() && offx.empty()) {
      offx_scaled = cv::Mat(base_x.size(), CV_16SC1, cv::Scalar(0));
      offy.convertTo(offy_scaled, CV_16SC1, getSignal("scaley"));
    } else {
      offx.convertTo(offx_scaled, CV_16SC1, getSignal("scalex"));
      offy.convertTo(offy_scaled, CV_16SC1, getSignal("scaley"));
    }*/

    cv::Mat out;
  #if 1
    cv::Mat offx8, offy8;
    cv::cvtColor(offx, offx8, CV_RGB2GRAY);
    cv::cvtColor(offy, offy8, CV_RGB2GRAY);
     
    // this may be more intuitive with flipped sign, 
    // currently offx is where the source pixel comes from,
    // (the opposite of positive numbers pushing pixels to the right)
    offx8.convertTo(offx_scaled, CV_32FC1, scalex);
    offy8.convertTo(offy_scaled, CV_32FC1, scaley);
    
    cv::Mat dist_x = base_x + (offx_scaled - offsetx * scalex);
    cv::Mat dist_y = base_y + (offy_scaled - offsety * scaley);
  
    cv::Mat dist_xy16, dist_int;
    cv::convertMaps(dist_x, dist_y, dist_xy16, dist_int, CV_16SC2, true);

    cv::remap(in, out, dist_xy16, cv::Mat(), getModeType(), getBorderType());
    #else
    cv::remap(in, out, base_x, base_y, getModeType() );
    #endif
    setImage("out", out);

    return true;
  }

///////////////////////////////////////////////////////////
  Tap::Tap(const std::string name) : ImageNode(name)
  {
    vcol = cv::Scalar(100, 30, 250);

    getSignal("value");
    getBuffer("buffer",0);
    setInputPort(BUFFER,"buffer", NULL, "out");
    //getImage("Buffer");
  }

  void Tap::setup(Signal* new_signal, Buffer* new_buffer) 
  {
    // TBD need caller to provide these
    setInputPort(SIGNAL,"value", new_signal, "value");
    setInputPort(BUFFER,"buffer", new_buffer, "out");
  }

  bool Tap::update()
  {
    if (!Node::update()) return false;

    if (isDirty(this,4)) {
      float value = getSignal("value");     
      
      VLOG(5) << name << " update " << value;
      cv::Mat out; // = getImage("out");
      int actual_ind;
      out = getBuffer("buffer", value, actual_ind); //, tmp)) return false;
      setSignal("actual_ind", actual_ind);
      
      if (out.empty()) return false;

      setImage("out", out);
    }

    return true;
  }

  bool Tap::draw(cv::Point2f ui_offset) 
  {
    ImageNode::draw(ui_offset);
  }
  
  bool TapInd::update()
  {
    if (!Node::update()) return false;

    if (isDirty(this, 4)) {
      float value =  getSignal("value");     
      int ind = value;

      VLOG(2) << name << " update " << ind;
      cv::Mat out; //= getImage("out");
      int actual_ind;
      out = getBuffer("buffer", ind, actual_ind);
      setSignal("actual_ind", actual_ind);
      if (out.empty())  return false;

      setImage("out", out);
    }

    return true;
  }
  
  bool TapInd::draw(cv::Point2f ui_offset) 
  {
    Tap::draw(ui_offset);
    return true;
  }

  ///////////////////////////////////////////
  Add::Add(const std::string name) : 
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("add0", tmp);
    setSignal("add0", 1.0);
    vcol = cv::Scalar(200, 200, 50);
  }
  
  void Add::setup(vector<ImageNode*> np, vector<float> nf) 
  {
    if (np.size() != nf.size()) {
      LOG(ERROR) << CLWRN << "mismatched inputs and coefficients" << CLNRM;
      //return; 
    }
   
    // TBD instead of clearing all, only clear the keys that match "add"
    for (int i = 0; i < np.size(); i++) {
      const string port = "add" + boost::lexical_cast<string>(i);
      setInputPort(IMAGE, port, np[i], "out");
      setInputPort(SIGNAL, port, NULL, "value"); // this allows other signals to connect to replace nf
    }
  }

  bool Add::update()
  {
    if (!Node::update()) return false;

    //VLOG(1) << "name " << is_dirty << " " << p1->name << " " << p1->is_dirty << ", " << p2->name << " " << p2->is_dirty ;
    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      // TBD accomodate bad mats somewhere

      cv::Size sz;

      bool done_something = false;
      
      cv::Mat out;

      // TBD should these vectors just be stored with some incrementing string?
      // TBD loop through all input ImageNodes and input signals (or vector values, need to be able
      // to handle either)
      // TBD instead of having nf, loop through all svals and match on ones that start with add
      // and then run getSignal on those strings
      for (int i = 0; i < ports.size(); i++) {
        if (ports[i]->type != IMAGE) continue;

        const string port = ports[i]->name;
        
        if (port.substr(0,3) != "add") {
          VLOG(5) << name << " : " << port.substr(0,3) << " " << port;
          continue;
        }
        
        cv::Mat tmp_in;
        bool im_dirty;
        //const string port = "add" + boost::lexical_cast<string>(i);
        tmp_in = getImage(port);
        if (tmp_in.empty()) {
          VLOG(2) << name << " : " << port << " image is empty"; 
          continue;
        }

        float val = getSignal(port);
        
        // with 8-bit unsigned it is necessary to have initial coefficients positive
        // zero minus anything will just be zero 
        if (!done_something) {
          out = tmp_in * val;
          sz = tmp_in.size();
          done_something = true;
        } else { 

          if (sz != tmp_in.size()) {
            LOG(ERROR) << name << " size mismatch " << sz.width << " " << sz.height 
                << " != " << tmp_in.size().width << " " << tmp_in.size().height ;
            continue;
          }
         
          if (val > 0)
            out += tmp_in * val;
          else 
            out -= tmp_in * -val;

        }
      } // nf loop
    
    //VLOG(1) << name << " " << "update";
    setImage("out", out);

    return true;
  }

  // TBD am duplicating this code a lot
  bool Add::handleKey(int key)
  {
    bool valid_key = ImageNode::handleKey(key);
    if (valid_key) return true;
   
    valid_key = true;
    if (key == '[') {
    
      // add an input addition port, TBD move to function
      int add_num = 0;
      for (int i = 0; i < ports.size(); i++) {
        if (ports[i]->type != IMAGE) continue;
        const string port = ports[i]->name;
        
        if (port.substr(0,3) != "add") {
          VLOG(1) << name << " : " << port.substr(0,3) << " " << port;
          continue;
        }
        add_num++;
      }

      // add a new addition port
      const string port = "add" + boost::lexical_cast<string>(add_num);
      setInputPort(IMAGE, port, NULL, "out");
      setInputPort(SIGNAL, port, NULL, "value"); // this allows other signals to connect to replace nf
       
      // TBD make a way to delete a port
    } else {
      valid_key = false;
    }

    // TBD 
    if (valid_key) setDirty();
    
    return valid_key;
  }

  //
  AddMasked::AddMasked(const std::string name) : 
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("add0", tmp);
    //setSignal("add0", 1.0);
    setImage("add1", tmp);
    //setSignal("add1", 1.0);
    setImage("mask", tmp);
    setSignal("offset", 0, false, SATURATE, 0, 255);
    vcol = cv::Scalar(200, 200, 50);
  }
 
  bool AddMasked::update()
  {
    if (!Node::update()) return false;

    //VLOG(1) << "name " << is_dirty << " " << p1->name << " " << p1->is_dirty << ", " << p2->name << " " << p2->is_dirty ;
    if (!isDirty(this, 5)) { 
      VLOG(4) << name << " not dirty ";
      return true; 
    }

    cv::Mat add0 = getImage("add0");
    if (add0.empty()) return true;
    cv::Mat add1 = getImage("add1");
    if (add1.empty()) return true;

    cv::Mat mask4 = getImage("mask");
    // derive the mask from the second input
    if (mask4.empty()) {
      // probably want to change add1 to black and white and then use it here,
      // otherwise color channels get masked individually. (a pixel that is 0 in red
      // but not in green will have different masks on those channels)
      mask4 = add1;
    } 
    
    int offset = getSignal("offset");
    cv::Mat mask = mask4 - cv::Scalar(offset,offset,offset,0); // cv::Mat(mask4.size(), CV_8UC1);
    // TBD use first channel as mask, TBD could combine all channels
    
    // this is masking individually on all color channels, probably
    cv::Mat out = add0 & (mask == 0);
    out += add1 & (mask > 0);
    //cv::Add(add0,
    setImage("out", out);
    return true;
  }


  ////////////////////////////////////////
  Multiply::Multiply(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("mul0", tmp);
    setImage("mul1", tmp);
    setSignal("mul0", 1.0/16.0);
    setSignal("mul1", 1.0/16.0);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool Multiply::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      
    // TBD accomodate bad mats somewhere
    cv::Size sz;

    bool done_something = false;
      
    cv::Mat out;

    for (int i = 0 ; i < ports.size(); i++) {
      if (ports[i]->type != IMAGE) continue;
      const string port = ports[i]->name;

      if (port.substr(0,3) != "mul") {
        VLOG(1) << name << " : " << port.substr(0,3) << " " << port;
        continue;
      }

      cv::Mat tmp_in;
      bool im_dirty;
      tmp_in = getImage(port);
      if (tmp_in.empty()) {
        continue;
      }

      float val = getSignal(port);
      if (val < 0) {
        val = 0;
        setSignal(port, val);
      }
      
      if (!done_something) {
        out = tmp_in * val;
        sz = tmp_in.size();
        done_something = true;
      } else { 

        if (sz != tmp_in.size()) {
          LOG(ERROR) << name << " size mismatch " << sz.width << " " << sz.height 
            << " != " << tmp_in.size().width << " " << tmp_in.size().height ;
          continue;
        }

        out = out.mul(tmp_in * val);
      }
    } // port loop

    setImage("out", out);

    return true;
  }

  ////////////////////////////////////////
  AbsDiff::AbsDiff(const std::string name)  : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("diff0", tmp);
    setImage("diff1", tmp);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool AbsDiff::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      
    cv::Mat out;
    
    cv::Mat diff0 = getImage("diff0");
    cv::Mat diff1 = getImage("diff1");

    if (diff0.empty() && !diff1.empty()) {
      out = diff1;
    } else if (!diff0.empty() && !diff1.empty()) {
      out = diff0;
    } else if (diff0.size() != diff1.size()) {
      LOG(ERROR) << name << " size mismatch";
      return false;
    }

    cv::absdiff(diff0, diff1, out);

    setImage("out", out);

    return true;
  }

  Max::Max(const std::string name)  : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in0", tmp);
    setImage("in1", tmp);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool Max::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
    
    cv::Mat in0 = getImage("in0");
    cv::Mat in1 = getImage("in1");

    cv::Mat out;
    {
      boost::mutex::scoped_lock l(port_mutex);

      if (in0.empty() && in1.empty()) {
        return false;
      } else if (in0.empty() && !in1.empty()) {
        out = in1;
      } else if (!in0.empty() && in1.empty()) {
        out = in0;
      } else if (in0.size() != in1.size()) {
        LOG(ERROR) << name << " size mismatch";
        return false;
      } else if (in0.type() != in1.type()) {
        return false;
      } else if (in0.size() == cv::Size(0,0)) {
        return false;
      } else {
        // TBD allow offsets, scales?
        out = cv::max(in0, in1);
      }
    }   
    setImage("out", out);

    return true;
  }


  ////////////////////////////////////////
  Greater::Greater(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in0", tmp);
    setImage("in1", tmp);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool Greater::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      
    cv::Mat out;
    
    cv::Mat diff0 = getImage("in0");
    cv::Mat diff1 = getImage("in1");
    {
    boost::mutex::scoped_lock l(port_mutex);

    if (diff0.empty() && diff1.empty()) {
      return false;
    } else if (diff0.empty() && !diff1.empty()) {
      out = diff1;
    } else if (!diff0.empty() && diff1.empty()) {
      out = diff0;
    } else if (diff0.size() != diff1.size()) {
      LOG(ERROR) << name << " size mismatch";
      return false;
    } else if (diff0.type() != diff1.type()) {
      return false;
    } else if (diff0.size() == cv::Size(0,0)) {
      return false;
    } else {
      out = diff0 > diff1;
    }
    }
    /*
      TBD use actual function and have an int  input select among these options
CMP_EQ src1 equal to src2.
CMP_GT src1 greater than src2.
CMP_GE src1 greater than or equal to src2.
CMP_LT src1 less than src2.
CMP_LE src1 less than or equal to src2.
CMP_NE
      Or simply have a key that switches in and ref 
    */

    setImage("out", out);

    return true;
  }


  ////////////////////////////////////////
  Resize::Resize(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("fx", 0.2);
    setSignal("fy", 0.2);
    setSignal("mode", 0, false, ROLL, 0, 4);
  }

  bool Resize::update()
  {
  if (!Node::update()) return false;
 
  if (!isDirty(this, 5)) { 
      VLOG(2) << name << " not dirty ";
      return true; 
  }
  cv::Mat in = getImage("in");

  if (in.empty()) {
    VLOG(2) << name << " in is empty";
    return false;
  }
  
  // if fx and fy aren't hooked up then they will remain unaltered

  cv::Size sz = in.size();
  
  float fx = abs(getSignal("fx"));
  float fy = abs(getSignal("fy"));
  
  cv::Size dsize = cv::Size(fx*sz.width, fy*sz.height);
  //TBD
  if (fx > 1.0) dsize.width = sz.width/fx;
  if (fy > 1.0) dsize.height = sz.height/fy;

  if (dsize.width < 1) dsize.width = 1;
  if (dsize.height < 1) dsize.height = 1;
  // TBD
  if (dsize.height > sz.height) dsize.height= sz.height;
  if (dsize.width > sz.width) dsize.width = sz.width;

  // scale image down
  cv::Mat tmp;
  cv::resize(in, tmp, dsize, 0, 0, cv::INTER_NEAREST);
  setSignal("sz_x", dsize.width);
  setSignal("sz_y", dsize.height);
  // then scale back to input size
  cv::Mat out;

  int mode_type = getModeType();
  
  // scale it back up to standard size
  cv::resize(tmp, out, sz, 0, 0, mode_type);
  setImage("out", out);
  VLOG(1) << fx << " " << fy << " " 
      << tmp.size().width << " " << tmp.size().height
      << " " << out.size().width << " " << out.size().height;
  return true;
  }

  ////////////////////////////////////////
  Flip::Flip(const std::string name) :
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("flip_code", 0);
  }

  bool Flip::update()
  {
    if (!Node::update()) return false;

    // TBD make sure keyboard changed parameters make this dirty 
    if (!isDirty(this, 5)) { 
      VLOG(4) << name << " not dirty ";
      return true; 
    }

    cv::Mat in = getImage("in");

    if (in.empty()) {
      VLOG(2) << name << " in is empty";
      return false;
    }

    // if fx and fy aren't hooked up then they will remain unaltered

    cv::Size sz = in.size();

    int flip_code = getSignal("flip_code");
    while (flip_code > 2) flip_code -= 3;
    while (flip_code <-1) flip_code += 3;
    setSignal("flip_code", flip_code);

    cv::Mat out = cv::Mat(sz, in.type());

    if (flip_code == 2) out = in;
    else {
      flip(in, out, flip_code);
    }

    setImage("out", out);

    return true;
  }


  ////////////////////////////////////////
  EqualizeHist::EqualizeHist(const std::string name) :
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setImage("mask", tmp);
    setSignal("hsv_ind", 2, false, ROLL, 0, 2);
  }

  bool EqualizeHist::update()
  {
    if (!Node::update()) return false;

    // TBD make sure keyboard changed parameters make this dirty 
    if (!isDirty(this, 5)) { 
      VLOG(4) << name << " not dirty ";
      return true; 
    }

    cv::Mat in = getImage("in");

    if (in.empty()) {
      VLOG(2) << name << " in is empty";
      return false;
    }
    
    cv::Mat mask4 = getImage("mask");

    if (!mask4.empty()) {
      cv::Mat mask = cv::Mat( mask4.size(), CV_8UC1);
      
      cv::cvtColor(mask4, mask, CV_BGR2GRAY);
      cv::Scalar masked_mean = cv::mean(in, mask); //[0];
      setSignal("m_b", masked_mean[0]); 
      setSignal("m_g", masked_mean[1]); 
      setSignal("m_r", masked_mean[2]); 
      // does setting the unmasked areas to the masked mean 
      // make the histogram equalization functionally masked?
      in = in.clone();
      //in += masked_mean & (mask4 == 0); 
      in += cv::Scalar::all(getSignal("black")) & (mask4 == 0);
      // TBD making so much of the image a single color produces a harsh light to dark transition
      // from equalizeHist, really it needs to be a gradient with the same histogram as the masked area
      //  
    }

    // change to CV_8UC1 and then equalize on the value channel
    cv::Mat hsv;
    // the fourth channel is lost in this conversion, hsv will be 3 channels
    cv::cvtColor(in, hsv, CV_BGR2HSV);

    cv::Mat in_v = cv::Mat( in.size(), CV_8UC1);
    const int ind = getSignal("hsv_ind");
    const int ch1[] = {ind, 0};
    mixChannels(&hsv, 1, &in_v, 1, ch1, 1);

    cv::Mat out_v; // = in_v.clone();
    cv::equalizeHist(in_v, out_v);

    const int ch2[] = {0, ind};
    mixChannels(&out_v, 1, &hsv, 1, ch2, 1);
    
    cv::Mat out_3;
    cv::cvtColor(hsv, out_3, CV_HSV2BGR);

    cv::Mat out = chan3to4(out_3);

    if (!mask4.empty()) {
      // now restore the masked out areas... this doesn't seem like it will
      // work, probably need to work with calcHist and apply histograms manually 
      
    }

    setImage("out", out);

    return true;
  }

  ////////////////////////////////////////
  Normalize::Normalize(const std::string name) :
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setImage("mask", tmp);
    setSignal("type", 0, false, ROLL, 0, 3);
    setSignal("alpha", 0);
    setSignal("beta", 255);
  }

  bool Normalize::update()
  {
    if (!Node::update()) return false;

    // TBD make sure keyboard changed parameters make this dirty 
    if (!isDirty(this, 5)) { 
      VLOG(4) << name << " not dirty ";
      return true; 
    }

    cv::Mat in = getImage("in");

    if (in.empty()) {
      VLOG(2) << name << " in is empty";
      return false;
    }
    
    cv::Mat mask4 = getImage("mask");

    cv::Mat mask;
    if (!mask4.empty()) {
      mask = cv::Mat( mask4.size(), CV_8UC1);
      cv::cvtColor(mask4, mask, CV_BGR2GRAY);
    }

    int type = getSignal("type");

    int norm_type = NORM_MINMAX;
    if (type == 1) norm_type = NORM_INF;
    if (type == 2) norm_type = NORM_L1;
    if (type == 3) norm_type = NORM_L2;

    cv::Mat out;

    cv::normalize(in, out, getSignal("alpha"), getSignal("beta"), norm_type, -1, mask);

    setImage("out", out);


    return true;

  }

  ////////////////////////////////////////
  Distance::Distance(const std::string name) :
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("threshold", 128);
    //setImage("mask", tmp);
    setSignal("type", 0, false, ROLL, 0, 2);
    setSignal("alpha", 1);
    setSignal("beta", 0);
    setSignal("make_labels", 0, false, ROLL, 0, 1);
    setImage("labels", tmp, true);
    setSignal("label_type", 0, false, ROLL, 0, 1);
  }

  bool Distance::update()
  {
    if (!Node::update()) return false;

    // TBD make sure keyboard changed parameters make this dirty 
    if (!isDirty(this, 5)) { 
      VLOG(4) << name << " not dirty ";
      return true; 
    }

    cv::Mat in = getImage("in");

    if (in.empty()) {
      VLOG(2) << name << " in is empty";
      return false;
    }
    
    cv::Mat mask;
    cv::cvtColor(in > getSignal("threshold"), mask, CV_BGR2GRAY);

    int type = getSignal("type");

    int distance_type = CV_DIST_L1;
    if (type == 1) distance_type = CV_DIST_L2;
    if (type == 2) distance_type = CV_DIST_C;

    cv::Mat out32;
    if (getSignal("make_labels") < 0.5) {
      cv::distanceTransform(mask, out32, distance_type, 3);
    } else {
      cv::Mat labels32;
      
      int label_ind = getSignal("label_type");
      int label_type = DIST_LABEL_CCOMP;
      if (label_ind == 1) label_type = DIST_LABEL_PIXEL;
      // TBD these labels could be useful for ContourFlip so it doesn't have to use Contours-
      // how do the labels get matched to their pixel coordinates though?
      cv::distanceTransform(mask, out32, labels32, distance_type, 3, label_type);

      cv::normalize(labels32, labels32, 0, 255, NORM_MINMAX);
      cv::Mat labels8;
      labels32.convertTo(labels8, CV_8UC1);
      cv::Mat labels = chan1to4(labels8);

      setImage("labels", labels);
    }

    cv::Mat out8;
    out32.convertTo(out8, CV_8UC1, getSignal("alpha"), getSignal("beta") );
  
    cv::Mat out = chan1to4(out8); 

    setImage("out", out);

    return true;
  }

  /////////////////////////////////////////////////////////
  DistanceFlip::DistanceFlip(const std::string name) :
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("to_flip", tmp);
    setImage("to_threshold", tmp);
    setSignal("threshold", 128);
    //setImage("mask", tmp);
    setSignal("type", 0, false, ROLL, 0, 2);
    setSignal("alpha", 1);
    setSignal("beta", 0);
    setImage("labels", tmp, true);
    setImage("dist", tmp, true);
  }

  bool DistanceFlip::update()
  {
    const bool rv = Node::update();
    if (!rv) return false;

    // TBD make sure keyboard changed parameters make this dirty 
    if (!isDirty(this, 5)) { 
      VLOG(4) << name << " not dirty ";
      return true; 
    }

    cv::Mat to_flip = getImage("to_flip");

    if (to_flip.empty()) {
      VLOG(2) << name << " in is empty";
      return false;
    }

    cv::Mat to_threshold = getImage("to_threshold");

    if (to_threshold.empty()) {
      VLOG(2) << name << " in is empty";
      return false;
    }
  
    //cv::Mat flipped = cv::Mat(to_flip.size(), to_flip.type(), cv::Scalar(128,0,64,0));
    cv::Mat flipped = cv::Mat(to_flip.size(), CV_8UC4, cv::Scalar(128,0,64,0));

    cv::Mat mask;
    cv::cvtColor(to_threshold > getSignal("threshold"), mask, CV_BGR2GRAY);

    int type = getSignal("type");

    int distance_type = CV_DIST_L1;
    if (type == 1) distance_type = CV_DIST_L2;
    if (type == 2) distance_type = CV_DIST_C;

    cv::Mat dist32;
    cv::Mat labels32;
    cv::distanceTransform(mask, dist32, labels32, distance_type, 3, DIST_LABEL_PIXEL);
    {  
      // for display, make optional if it cost much?
      cv::Mat labels32b;
      cv::normalize(labels32, labels32b, 0, 255, NORM_MINMAX);
      cv::Mat labels8;
      labels32b.convertTo(labels8, CV_8UC1);
      cv::Mat labels = chan1to4(labels8);
      setImage("labels", labels);
    }

    {
      cv::Mat dist8, dist32b;
      //dist32.convertTo(dist8, CV_8UC1, getSignal("alpha"), getSignal("beta") );
      cv::normalize(dist32, dist32b, 0, 255, NORM_MINMAX);
      dist32b.convertTo(dist8, CV_8UC1, getSignal("alpha"), getSignal("beta") );
      cv::Mat dist = chan1to4(dist8); 
      setImage("dist", dist);
    }

    const int wd = labels32.cols;
    const int ht = labels32.rows;

    // make a mapping between label values and pixel coordinates,
    // it would be nice if distanceTransform did this or just provided
    // pixel coordinates to begin with and the following would work:
    // const int lx = label % wd;
    //  const int ly = label / wd;
    std::map<int, cv::Point> label_map;
    int k = 1;
    for (int y = 0; y < ht; y++) {
    for (int x = 0; x < wd; x++) {
      if (mask.at<uchar>(y,x) == 0) {
        label_map[k] = cv::Point(x,y);
        k++;
      }
    }}

    // loop through every pixel, if it is labeled with itself do nothing, but 
    // if the label at the location is different then use the pixel that is the same 
    // distance away on the opposite side of the label pixel.
    for (int y = 0; y < ht; y++) {
    for (int x = 0; x < wd; x++) {

      const int ind = y * wd + x;
      const int label = labels32.at<int>(y,x);
      //if (label == ind) continue;
     
      // don't actually need the distance
      //const float distance = dist.at<float>(y,x);

      map<int, cv::Point >::iterator label_it;
      label_it = label_map.find(label);
      if (label_it == label_map.end()) {
        // TBD use current pixel
        //LOG(ERROR) << label << " not in map " << x << " " << y;
        continue;
      }

      cv::Point pos = label_map[label]; 

      // TBD make a reflection effect instead of straight rolling over the edges?
      const int src_x = ((x + (int) ( 2 * (pos.x - x) ) ) + wd) % wd;
      const int src_y = ((y + (int) ( 2 * (pos.y - y) ) ) + ht) % ht;
    
      //if (VLOG_IS_ON(1) ) {
      LOG_FIRST_N(INFO, wd*2) 
          << y     << " " << x     << " " << ind << ",\t" 
          << pos.y    << " " << pos.x    << " " << label << ",\t" 
          << src_y << " " << src_x << ",\t" 
          << ind%255;
      //}
             
      flipped.at<cv::Vec4b>(y, x) = to_flip.at<cv::Vec4b>(src_y, src_x);
      //flipped.at<cv::Vec4b>(y, x) = to_flip.at<cv::Vec4b>(y, x);

      // flipped.at<cv::Vec4b>(y, x) = cv::Scalar::all(label%255); //to_flip.at<cv::Vec4b>(src_y, src_x);
      // flipped.at<cv::Vec4b>(y, x) = cv::Scalar::all(ind%255); //cv::Vec4b(ind%255, label%255, ind%255, 0); //to_flip.at<cv::Vec4b>(src_y, src_x);
      // flipped.at<cv::Vec4b>(src_y, src_x) = to_flip.at<cv::Vec4b>(y,x); //cv::Scalar(ind%255, label%255, 0, 0); //to_flip.at<cv::Vec4b>(src_y, src_x);

    }}

    setImage("out", flipped);

    return true;
  }

  ////////////////////////////////////////
  FloodFill::FloodFill(const std::string name) :
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("r", 128, false, SATURATE, 0, 255);
    setSignal("g", 128, false, SATURATE, 0, 255);
    setSignal("b", 128, false, SATURATE, 0, 255);
    //setImage("mask_in", tmp);  // TBD
    
    // normalized 0-10
    setSignal("x", 5, false, ROLL, 0, 10);
    setSignal("y", 5, false, ROLL, 0, 10);
    setSignal("lodiff", 10); //, false, ROLL, 0, 2);
    setSignal("hidiff", 10);
    
    setSignal("mask_mode", 0, false, ROLL, 0, 1);
    setSignal("fill_mode", 0, false, ROLL, 0, 1);
  }

  bool FloodFill::update()
  {
    if (!Node::update()) return false;

    // TBD make sure keyboard changed parameters make this dirty 
    if (!isDirty(this, 5)) { 
      VLOG(4) << name << " not dirty ";
      return true; 
    }

    cv::Mat in = getImage("in");

    if (in.empty()) {
      VLOG(2) << name << " in is empty";
      return false;
    }
  
    cv::Point seed_pt = cv::Point(
        getSignal("x")/10.0 * Config::inst()->getImSize().width,
        getSignal("y")/10.0 * Config::inst()->getImSize().height
        );
    cv::Scalar newval = cv::Scalar(
        getSignal("r"),
        getSignal("g"),
        getSignal("b")
        );

    int connectivity = 4; // also could be 8
    int ffillMode = getSignal("fill_mode") < 0.5;
    // newMaskVal is not really documented but is the single channel value of
    // what to output in the FLOODFILL_MASK_ONLY option
    const int newMaskVal = 255;
    const int flags = connectivity + 
        (newMaskVal << 8) +   
        (ffillMode == 1 ? CV_FLOODFILL_FIXED_RANGE : 0);
    
    cv::Rect ccomp;
    // TBD clone may not be necessary
    cv::Mat out_3 = chan4to3(in).clone(); //cv::Mat(in.size(), CV_8UC3); //in.clone();

    if (getSignal("mask_mode") > 0.5) {
      
      cv::Mat mask = cv::Mat( cv::Size(in.cols + 2, in.rows + 2), CV_8UC1, cv::Scalar::all(0) ); 
      cv::floodFill(
        out_3,
        mask,
        seed_pt, 
        newval, 
        &ccomp,
        cv::Scalar::all((int)getSignal("lodiff")),
        cv::Scalar::all((int)getSignal("hidiff")),
        flags | FLOODFILL_MASK_ONLY);

      cv::Mat out_1 = mask(cv::Rect(1, 1, in.cols, in.rows));
      //cv::Mat out_1 = cv::Mat(in.size(), CV_8UC1, cv::Scalar::all(200));

      cv::Mat out = chan1to4(out_1);

      setImage("out", out);

    } else {

      cv::floodFill(
        out_3, 
        seed_pt, 
        newval, 
        &ccomp,
        cv::Scalar::all((int)getSignal("lodiff")),
        cv::Scalar::all((int)getSignal("hidiff")),
        flags);

      cv::Mat out = chan3to4(out_3);
      setImage("out", out);

    }

    return true;
  }   

} //bm

