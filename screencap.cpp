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


#include "screencap.h"
#include "config.h"

#include <glog/logging.h>

#include "utility.h"

using namespace std;

namespace bm {

ScreenCap::ScreenCap(const std::string name) :ImageNode(name)
{
  display = NULL;
  screen = NULL;
  xImageSample = NULL;

  display = XOpenDisplay(NULL); // Open first (-best) display
  if (display == NULL) {
    LOG(ERROR) << name << " bad display";
    return;
  }

  screen = DefaultScreenOfDisplay(display);
  if (screen == NULL) {
    LOG(ERROR) << name << " bad screen";
    return;
  }

  Window wid = DefaultRootWindow( display );
  if ( 0 > wid ) {
    LOG(ERROR) << "Failed to obtain the root windows Id "
        "of the default screen of given display.\n";
    return;
  }

  XWindowAttributes xwAttr;
  Status ret = XGetWindowAttributes( display, wid, &xwAttr );
  screen_w = xwAttr.width;
  screen_h = xwAttr.height;

  LOG(INFO) << name << " screen w h " << CLVAL << screen_w << " " << screen_h << CLNRM;
  // The starting pixels / offset (x,y) to take the screenshot from
  int startX = 0;
  int startY = 0;

  // The size of the area (width,height) to take a screenshot of
  int widthX = Config::inst()->im_width; //screen->width / 4, heightY = screen->height / 4; 
  int heightY = Config::inst()->im_height;
  
  cv::Mat out = cv::Mat(cv::Size(widthX, heightY), CV_8UC4);
  out = cv::Scalar(50,50,200);

  setSignal("startX", startX);
  setSignal("startY", startY);
  setSignal("widthX", widthX);
  setSignal("heightY", heightY);

  setImage("out", out);
}

bool ScreenCap::update()
{
  if (!Node::update()) return false;
  
  if ((display == NULL) || (screen == NULL)) { return false; }

  int startX  = getSignal("startX");
  int startY  = getSignal("startY");
  int widthX  = getSignal("widthX");
  int heightY = getSignal("heightY");

  if (startX < 0) startX = 0;
  if (startY < 0) startY = 0;

  // Need to check against resolution
  if ((startX + widthX) > screen_w) {
    // TBD need to more intelligently cap these
    if (screen_w > widthX) {
      startX = screen_w - widthX;
    } else {
      startX = 0;
      widthX = Config::inst()->im_width;
    }
  }

  if ((startY + heightY) > screen_h) {
    // TBD need to more intelligently cap these
    if (screen_h > heightY) {
      startY = screen_h - heightY;
    } else {
      startY = 0;
      heightY = Config::inst()->im_height;
    }
  }
 
  setSignal("startX", startX);
  setSignal("startY", startY);
  setSignal("widthX", widthX);
  setSignal("heightY", heightY);

  xImageSample = XGetImage(display, DefaultRootWindow(display), 
      startX, startY, widthX, heightY, AllPlanes, ZPixmap);
  
  // Check for bad null pointers
  if (xImageSample == NULL) { 
    LOG(ERROR) << "Error taking screenshot!";
    return false;
  }

  //    col.pixel = XGetPixel(image, x, y); // Get pixel at x,y
  cv::Mat tmp = XImage2OpenCVImage(*xImageSample, *display, *screen);

  // To get the color values of the (sub-)image (can be slow, there are alternatives)
  //    XQueryColor(display, DefaultColormap(display, DefaultScreen(display)), &col); 

  // Always clean up your mess
  XDestroyImage(xImageSample);
  //XCloseDisplay(display);

  if (tmp.empty()) return false;
 
  // resize
  cv::Size sz = cv::Size(Config::inst()->im_width, Config::inst()->im_height);
  cv::Mat tmp1;
  cv::resize(tmp, tmp1, sz, 0, 0, cv::INTER_NEAREST );
  setImage("out", tmp);

  setDirty(); 
 
  return true;
}

} // bm

