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
  
    cv::Mat in = getImage("in");
    if (in.empty()) return true;

    if (!ximage) return true;
   
    // TBD is this necessary or does X do it for me if the window is resized?
    cv::Size sz = cv::Size(ximage->width, ximage->height);
    cv::Mat scaled;
    cv::resize(in, scaled, sz, 0, 0, cv::INTER_NEAREST );
    bm::matToXImage(scaled, ximage, win, *display, *screen);
  }

  bool Output::draw()
  {
    /*
    XWindowAttributes xwAttr;
    Status ret = XGetWindowAttributes( display, win, &xwAttr );
    int screen_w = xwAttr.width;
    int screen_h = xwAttr.height;


    if (ximage) XPutImage(display, win,  gc, ximage, 0, 0, 0, 0, screen_w, screen_h);
    */
    if (ximage) XPutImage(display, win,  gc, ximage, 0, 0, 0, 0, ximage->width, ximage->height);
    return ImageNode::draw();
  }

}

