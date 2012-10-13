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

#include "output.h"

#include <boost/timer.hpp>
#include <glog/logging.h>

#include "camthing.h"
#include "utility.h"

namespace bm {

  Output::Output() :
    ximage(NULL),
    display(NULL)
  { cv::Mat out; setImage("in",out);
  }

  bool Output::setup(const int width, const int height)
  {
    bm::setupX(display, win, width, height, opcode);
    gc = XCreateGC(display, win, 0, NULL);
    ximage = XGetImage(display, DefaultRootWindow(display), 0, 0, width, height, AllPlanes, ZPixmap);
    screen = DefaultScreenOfDisplay(display);
  }

  bool Output::update()
  {
    if (!ImageNode::update()) return false;
  }

  bool Output::draw(cv::Point2f ui_offset)
  {
    boost::timer t1;

    bool window_decorations_on = getSignal("decor");
    setSignal("decor", window_decorations_on);
    bm::setWindowDecorations(display, win, window_decorations_on);

    VLOG(3) << "decor draw time" << t1.elapsed(); 
    /*
    XWindowAttributes xwAttr;
    Status ret = XGetWindowAttributes( display, win, &xwAttr );
    int screen_w = xwAttr.width;
    int screen_h = xwAttr.height;


    if (ximage) XPutImage(display, win,  gc, ximage, 0, 0, 0, 0, screen_w, screen_h);
    */
 
    cv::Mat in = getImage("in");
    //if (in.empty()) return true;

    if (!(in.empty()) && ximage) {

    XWindowAttributes xwAttr;
    Status ret = XGetWindowAttributes( display, win, &xwAttr );
    int screen_w = xwAttr.width;
    int screen_h = xwAttr.height;
    setSignal("disp_w", screen_w);
    setSignal("disp_h", screen_h);
    
    VLOG(4) << "attr time" << t1.elapsed(); 

    // TBD is this necessary or does X do it for me if the window is resized?
    const cv::Size sz = cv::Size(screen_w, screen_h);
    cv::Mat scaled;
    if (sz == in.size()) 
      scaled = in.clone();
    else
      cv::resize(in, scaled, sz, 0, 0, cv::INTER_NEAREST );
   
    VLOG(5) << "resize time" << t1.elapsed();

    XDestroyImage(ximage);
    ximage = XGetImage(display, DefaultRootWindow(display), 0, 0, screen_w, screen_h, AllPlanes, ZPixmap);

    VLOG(4) << "get im time" << t1.elapsed();
    // this is slow
    bm::matToXImage(scaled, ximage, win, *display, *screen);
    VLOG(4) << "matToXImage time" << t1.elapsed();
    
    XPutImage(display, win,  gc, ximage, 0, 0, 0, 0, ximage->width, ximage->height);
    VLOG(3) << "put image time" << t1.elapsed();
    }

    return ImageNode::draw(ui_offset);
  }

}

