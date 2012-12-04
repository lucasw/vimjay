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

#include "output.h"

#include <boost/timer.hpp>
#include <glog/logging.h>

#include "config.h"
#include "utility.h"

namespace bm {

  Output::Output(const std::string name) :
    ImageNode(name),
    ximage(NULL),
    display(NULL)
  { 
    cv::Mat out; setImage("in",out);

    setSignal("decor",1, false, SATURATE, 0, 1); // TBD make a SATURATE_INTEGER, or ROLL_INTEGER type?
    setSignal("mode", 0, false, ROLL, 0, 4);

    setSignal("x", Config::inst()->ui_width, false, SATURATE, 0, 1e6); // TBD FLT_MAX instead of 1e6?
    setSignal("y", 0, false, SATURATE, 0, 1e6);
    setSignal("w", Config::inst()->out_width,  false, SATURATE, 1, 1e6);
    setSignal("h", Config::inst()->out_height, false, SATURATE, 1, 1e6);
  }

  bool Output::setup(const int width, const int height)
  {
    bm::setupX(display, win, width, height, opcode);
    gc = XCreateGC(display, win, 0, NULL);
    ximage = XGetImage(display, DefaultRootWindow(display), 0, 0, width, height, AllPlanes, ZPixmap);
    screen = DefaultScreenOfDisplay(display);
    XStoreName(display, win, name.c_str());
  }

  bool Output::update()
  {
    const bool rv = ImageNode::update();
    if (!rv) return false;
    // The threading issues with xwindows for the xwindows calls to be put
    // in the draw call
    return true;
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
 
    XWindowAttributes xwAttr, xwAttr2;
    // these don't seem to be updating x and y
    Window toplevel_parent = get_toplevel_parent(display, win);
    //Status ret = XGetWindowAttributes( display, win, &xwAttr );

    // this has the real x and y position
    Status ret = XGetWindowAttributes( display, toplevel_parent, &xwAttr );
    // this holds the 'true' width and height (the inner window not including
    // gnome decor?) the x and y are relative to the toplevel_parent
    Status ret2 = XGetWindowAttributes( display, win, &xwAttr2 );
      
    VLOG(6) << name  
        << " top  " << xwAttr.x << " " << xwAttr.y << ", " 
        << xwAttr.width << " " << xwAttr.height
        << " win " << xwAttr2.x << " " << xwAttr2.y << ", " 
        << xwAttr2.width << " " << xwAttr2.height;

    // TBD the user ought to be able to force x,y,w,h changes
    // the logic would be that if the window attributes disagree with the last
    // update, use those, if the getSignal value is different than the old signal,
    // that means the user wants the window resized
    
    int wm_x = xwAttr.x; // + xwAttr2.x;
    int wm_y = xwAttr.y; // + xwAttr2.y;

    if (
        (x != wm_x) ||
        (y != wm_y) ||
        (w != xwAttr2.width) ||
        (h != xwAttr2.height) 
       )
    {
      // don't do anything, the window is already set properly
      x = wm_x;
      y = wm_y;
      w = xwAttr2.width;
      h = xwAttr2.height;
      VLOG(6) << name << " wm changed display " 
        << x << " " << y << ", " << w << " " << h;

      setSignal("x", x);
      setSignal("y", y);
      setSignal("w", w);
      setSignal("h", h);

    } else {
      // see if user changed anything through vimjay interface,
      // this has lower priority than window manager changes
      int new_x = getSignal("x");
      int new_y = getSignal("y");
      int new_w = getSignal("w");
      int new_h = getSignal("h");
      
      if (
        (x != new_x) ||
        (y != new_y) ||
        (w != new_w) ||
        (h != new_h) 
        )
      {
        VLOG(6) << name << " vimjay changed display "
            << CLVAL << new_x << " " << new_y << ", " << new_w << " " << new_h << CLNRM
            << ", old " 
            << CLVAL << x << " " << y << ", " << w << " " << h << CLNRM;
        //int dx = new_x - x;
        //int dy = new_y - y;
        x = new_x;
        y = new_y;
        w = new_w;
        h = new_h;

        XWindowChanges values;
        unsigned int value_mask = 0x0;
        
        values.x = x;
        values.y = y;
        // if the window is crammed against the right border, increasing width
        // results in shrinking the window which is strange 
        values.width = w;
        values.height = h;
        
        VLOG(6) << values.x << " " << values.y;
        // TBD test if different
        {
          value_mask = CWWidth | CWHeight;
          value_mask |= CWX | CWY;
          XConfigureWindow( display, win, value_mask, &values);
        }

        // TBD if (win != toplevel_parent)
        { 
          // TBD hack, the difference between the toplevel window and the 
          // actual one are overcome by giving all of the offset to win zeroing
          // out the toplevel, when the wm changes the position it all goes to the toplevel.
          values.x = 0;
          values.y = 0;
          value_mask = CWX | CWY;
          XConfigureWindow( display, toplevel_parent, value_mask, &values);
        }
       
        setSignal("x", x);
        setSignal("y", y);
        setSignal("w", w);
        setSignal("h", h);

      } // see if vimjay ui change xywh

    } // xywh changes


    cv::Mat in = getImage("in");
    //if (in.empty()) return true;

    if (!(in.empty()) && ximage) {
    
    // These aren't used, just provided for convenience
    // and are set here to make sure the user didn't try to override them
    // (TBD set to be unwriteable when that functionality is supported
    setSignal("im_w", Config::inst()->im_width);
    setSignal("im_h", Config::inst()->im_height);
    
    VLOG(4) << "attr time" << t1.elapsed(); 

    // TBD is this necessary or does X do it for me if the window is resized?
    const cv::Size sz = cv::Size(w, h);
    cv::Mat scaled;
    if (sz == in.size()) 
      scaled = in.clone();
    else
      cv::resize(in, scaled, sz, 0, 0, getModeType() );
   
    VLOG(5) << "resize time" << t1.elapsed();

    XDestroyImage(ximage);
    ximage = XGetImage(display, DefaultRootWindow(display), 0, 0, w, h, AllPlanes, ZPixmap);
    
    VLOG(4) << "get im time" << t1.elapsed();
    // this is slow
    bm::matToXImage(scaled, ximage, win, *display, *screen);
    VLOG(4) << "matToXImage time" << t1.elapsed();
    
    XPutImage(display, win,  gc, ximage, 0, 0, 0, 0, ximage->width, ximage->height);
    VLOG(3) << "put image time" << t1.elapsed();
    } // ximage and input image is valid

    return ImageNode::draw(ui_offset);
  }

} // namespace bm

