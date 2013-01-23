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



#include "utility.h"

#include <boost/filesystem/operations.hpp>

#include "config.h"

#include <boost/timer.hpp>
#include <glog/logging.h>

#include <typeinfo>
#include <cxxabi.h> // non portable

namespace bm {


/**
  Provide a list of all the images (png or jpg) in a directory, 
  and also all the subdirectories.
*/
  bool getImageNamesAndSubDirs(const std::string dir, std::vector<std::string>& image_names, std::vector<std::string>& sub_dirs)
  {

    image_names.clear();
    sub_dirs.clear();

    boost::filesystem::path image_path(dir);
    if (!is_directory(image_path)) {
      LOG(ERROR) << CLERR << " not a directory " << CLNRM << dir;
      return false;
    }

    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    for ( boost::filesystem::directory_iterator itr( image_path );
        itr != end_itr;
        ++itr )
    {

      std::stringstream ss;
      ss << *itr;
      std::string str = ( ss.str() );
      // strip off "" at beginning/end
      str = str.substr(1, str.size()-2);

      if ( is_directory( *itr ) ) {
        sub_dirs.push_back( str );
        VLOG(1) << dir << " sub dir " << str;
      } else {
        if (!((str.substr(str.size()-3,3) != "jpg") ||
              (str.substr(str.size()-3,3) != "png"))
           ) {
          VLOG(1) << "not expected image type: " << str;
          continue;
        }

        image_names.push_back(str);
        VLOG(1) << dir << " image " << str; 
      }

    } //

  } // get image and sub dir names

  void initRemaps(cv::Mat& base_x, cv::Mat& base_y)
  {
    base_x = cv::Mat( Config::inst()->getImSize(), CV_32FC1);
    base_y = base_x.clone();

    for (int i = 0; i < base_x.rows; i++) {
      for (int j = 0; j < base_x.cols; j++) {
        base_x.at<float>(i,j) = j;
        base_y.at<float>(i,j) = i;
      }
    }
  }

  std::string getId(Node* ptr) 
  {
    int status; 
    return (abi::__cxa_demangle(typeid(*ptr).name(), 0, 0, &status));
  }

  std::string logMat(const cv::Mat& m) 
  {
    std::stringstream s;
    s << "{";
    for (int x = 0; x < m.rows; x++) {
      for (int y = 0; y < m.cols; y++) {
        s << " " << m.at<double>(x,y); 
      }
      if (x < m.rows-1) s << std::endl;
    }
    s << " }";
    return s.str();
  }

  bool getBezier(
      const std::vector<cv::Point2f>& control_points, // TBD currently has to be 4
      std::vector<cv::Point2f>& output_points,
      const int num // numbe of intermediate points to generate 
      )
  {
    if (control_points.size() != 4) {
      LOG(ERROR) << control_points.size() << " != 4"; 
      return false;
    }

    /*
    // 2nd order 1 2 1
    double coeff_raw[4][4] = {
      { 1, 0, 0},
      {-2, 2, 0},
      { 1,-2, 1},
    };
    // 4th order 1 4 6 4 1

    general pattern
    bc(1) =    1 1
    bc(2) =   1 2 1
    bc(3) =  1 3 3 1
    bc(4) = 1 4 6 4 1
    
    bc(3,0) = 1
    bc(3,1) = 3

    (1-x)(1-x)(1-x) = 1 -3x 3x^2 -x^3
    (1 -2x x^2) (1-x) 
    
    bc(+/-0) =   1
    bc(-1) =    1 -1
    bc(-2) =   1 -2  1
    bc(-3) =  1 -3  3 -1
    bc(-4) = 1 -4  6 -4  1 
    ...

      { bc(-3)*bc(3,0),  0               0               0
                        bc(-2)*bc(3,1)   0               0
                                         bc(-1)*bc(3,2)  0
                                                         bc(-0)*bc(3,3)

       1  0   0   0  0
      -4  4   0   0  0
       6 -12  6   0  0
      -4  12 -12  4  0
       1 -4   6  -4  1   

    */

    // TBD how to generate programmatically
    // 1 3 3 1
    double coeff_raw[4][4] = {
      { 1, 0, 0, 0},
      {-3, 3, 0, 0},
      { 3,-6, 3, 0},
      {-1, 3,-3, 1}
    };
    cv::Mat coeff = cv::Mat(4, 4, CV_64F, coeff_raw);
    cv::Mat control = cv::Mat::zeros(4, 2, CV_64F);
    
    for (int i = 0; i < control.rows; i++) {
      control.at<double>(i, 0) = control_points[i].x;
      control.at<double>(i, 1) = control_points[i].y;
    }

    VLOG(5) << CLTXT << "coeff " << CLNRM << std::endl << logMat(coeff); 
    VLOG(5) << CLTXT <<"control " << CLNRM << std::endl << logMat(control); 

    cv::Point2f old_pt;

    output_points.clear();

    for (int i = 0; i < num; i++) {
      float t = (float)i/(float)(num-1);

      // concentrate samples near beginning and end
      if (t < 0.5) {
        t *= t;
      } else {
        t = 1.0 - (1.0-t)*(1.0-t);
      }
      double tee_raw[1][4] = {{ 1.0, t, t*t, t*t*t}};

      cv::Mat tee = cv::Mat(1, 4, CV_64F, tee_raw);
      cv::Mat pos = tee * coeff * control;

      cv::Point new_pt = cv::Point2f(pos.at<double>(0,0), pos.at<double>(0,1));

      output_points.push_back(new_pt);

      VLOG(5) << "pos " << t << " "
        << new_pt.x << " " << new_pt.y 
        << std::endl << logMat(tee) 
        << std::endl << logMat(pos); 
    }

    return true;
  }


///////////////////////////////////////////////////////////////////////////////
// XWindows


/*
   Returns the parent window of "window" (i.e. the ancestor of window
   that is a direct child of the root, or window itself if it is a direct child).
   If window is the root window, returns window.

   From http://stackoverflow.com/questions/3909713/xlib-xgetwindowattributes-always-returns-1x1
   */
Window get_toplevel_parent(Display * display, Window window)
{
  Window parent;
  Window root;
  Window * children;
  unsigned int num_children;

  while (1) {
    if (0 == XQueryTree(display, window, &root,
          &parent, &children, &num_children)) {
      fprintf(stderr, "XQueryTree error\n");
      abort(); //change to whatever error handling you prefer
    }
    if (children) { //must test for null
      XFree(children);
    }
    if (window == root || parent == root) {
      return window;
    }
    else {
      window = parent;
    }
  }
}

bool setupX(Display*& display, Window& win, const int width, const int height, int& opcode) 
{
  // from git://gitorious.org/vinput/vinput.git demo-paint.c (GPL)
	/* Connect to the X server */
  display = XOpenDisplay(NULL);


	/* Check if the XInput Extension is available */
	int event, error;
	if (!XQueryExtension(display, "XInputExtension", &opcode, &event, &error)) {
		printf("X Input extension not available.\n");
		return false;
	}
  LOG(INFO) <<"XInputExtension available";

	/* Check for XI2 support */
	int major = 2, minor = 0;
	if (XIQueryVersion(display, &major, &minor) == BadRequest) {
		printf("XI2 not available. Server supports %d.%d\n", major, minor);
		return false;
	}
  LOG(INFO) <<"XI2 available";

	/* Set up MPX events */
	XIEventMask eventmask;
	unsigned char mask[1] = { 0 };

	eventmask.deviceid = XIAllMasterDevices;
	eventmask.mask_len = sizeof(mask);
	eventmask.mask = mask;

	/* Events we want to listen for */
	XISetMask(mask, XI_Motion);
	XISetMask(mask, XI_ButtonPress);
	XISetMask(mask, XI_ButtonRelease);
	//XISetMask(mask, XI_KeyPress);
	//XISetMask(mask, XI_KeyRelease);

	/* Register events on the window */
  int screen_num = DefaultScreen(display);
  win = XCreateSimpleWindow(display, DefaultRootWindow(display),
          0, 0, width, height, 5, 
          WhitePixel(display, screen_num), 
          BlackPixel(display, screen_num));
  XMapWindow(display, win);
  XSync( display, False );

  XISelectEvents(display, win, &eventmask, 1);

  if (0) {
  // move to upper corner?  No it moves it to the far left but a few menu bar widths below the top of the screen
  Window wid = DefaultRootWindow( display );
  XMoveResizeWindow(display, wid, 0,0, width, height);
  XMapRaised(display, wid);
  }

  {
    Window wid = DefaultRootWindow( display );
    XWindowAttributes xwAttr;
    Status ret = XGetWindowAttributes( display, wid, &xwAttr );
    int screen_w = xwAttr.width;
    int screen_h = xwAttr.height;

    LOG(INFO) << " screen w h " << CLVAL << screen_w << " " << screen_h << CLNRM;
    // This is the same as the attributes above
    LOG(INFO) << XDisplayWidth(display, 0) << " " << XDisplayHeight(display, 0);
    // this is junk
    LOG(INFO) << XDisplayWidth(display, 1) << " " << XDisplayHeight(display, 1);

    // TBD how to get resolution of particular monitor?

  }

  return true;
}

bool setWindowDecorations(Display* display, Window& win, bool decorations_on) 
{
    //code to remove decoration
  Hints hints;
  Atom property;
  hints.flags = 2;
  hints.decorations = decorations_on ? 1 : 0;
  property = XInternAtom(display, "_MOTIF_WM_HINTS", true);
  XChangeProperty(display, win, property, property, 32, PropModeReplace, (unsigned char *)&hints,5);
  XMapWindow(display, win);
}

// TBD may not need this function, user ought to keep ximage around and reuse it
bool matToScreen(cv::Mat& tmp, Display* display, Window& win) 
{
  if (tmp.empty()) return false;
  // TBD handle size mismatches
  const int width = tmp.cols;
  const int height = tmp.rows;
  
  // grabbing image for now, later 
  XImage* ximage = XGetImage(display, DefaultRootWindow(display), 0, 0, width, height, AllPlanes, ZPixmap);
  
  if (ximage == NULL) return false;

  Screen* screen = DefaultScreenOfDisplay(display);
  //XImage* ximage = XCreateImage(display, DefaultVisual(display, screen) 
  bm::matToXImage(tmp, ximage, win, *display, *screen);
  GC gc = XCreateGC(display, win, 0, NULL);
  XPutImage(display, win,  gc, ximage, 0, 0, 0, 0, width, height);
  
  XDestroyImage(ximage);

  return true;
}

/**
 * This is the function to demonstrate the conversion XImage to IplImage
 * @param ximage the X image object
 * @param _xDisplay the X display, for init see main() below
 * @param _xScreen the X screen, for init see main() below
 * @return
 */
cv::Mat XImage2OpenCVImage(XImage& ximage, Display& _xDisplay, Screen& _xScreen) 
{
    XColor color;
    //    cout << "WxH: " << imageData->width << "x" << imageData->height << ": " << imageData->data[0] << imageData->data[1] << imageData->data[2] << imageData->format << endl;
    //IplImage* ocvImage = cvCreateImage(cv::Size(ximage.width, ximage.height), IPL_DEPTH_8U, 3);
    cv::Mat tmp = cv::Mat(cv::Size(ximage.width, ximage.height), CV_8UC4);

    if (_xScreen.depths->depth == 24) {
        // Some of the following code is borrowed from http://www.roard.com/docs/cookbook/cbsu19.html ("Screen grab with X11" - by Marko Riedel, with an idea by Alexander Malmberg)
        unsigned long rmask = _xScreen.root_visual->red_mask,
                gmask = _xScreen.root_visual->green_mask,
                bmask = _xScreen.root_visual->blue_mask;
        unsigned long rshift, rbits, gshift, gbits, bshift, bbits;
        unsigned char colorChannel[3];

        rshift = 0;
        rbits = 0;
        while (!(rmask & 1)) {
            rshift++;
            rmask >>= 1;
        }
        while (rmask & 1) {
            rbits++;
            rmask >>= 1;
        }
        if (rbits > 8) {
            rshift += rbits - 8;
            rbits = 8;
        }

        gshift = 0;
        gbits = 0;
        while (!(gmask & 1)) {
            gshift++;
            gmask >>= 1;
        }
        while (gmask & 1) {
            gbits++;
            gmask >>= 1;
        }
        if (gbits > 8) {
            gshift += gbits - 8;
            gbits = 8;
        }

        bshift = 0;
        bbits = 0;
        while (!(bmask & 1)) {
            bshift++;
            bmask >>= 1;
        }
        while (bmask & 1) {
            bbits++;
            bmask >>= 1;
        }
        if (bbits > 8) {
            bshift += bbits - 8;
            bbits = 8;
        }

       const int wd = ximage.width;
       const int ht = ximage.height;
       memcpy(tmp.data, ximage.data, wd*ht*4); 
       #if 0
        for (unsigned int x = 0; x < ximage.width; x++) {
            for (unsigned int y = 0; y < ximage.height; y++) {
                color.pixel = XGetPixel(&ximage, x, y);
                colorChannel[0] = ((color.pixel >> bshift) & ((1 << bbits) - 1)) << (8 - bbits);
                colorChannel[1] = ((color.pixel >> gshift) & ((1 << gbits) - 1)) << (8 - gbits);
                colorChannel[2] = ((color.pixel >> rshift) & ((1 << rbits) - 1)) << (8 - rbits);
                cv::Vec4b col = cv::Vec4b(colorChannel[0], colorChannel[1], colorChannel[0],0);
                tmp.at<cv::Vec4b> (y,x) = col;
            }
        }
        #endif
    } else { // Extremly slow alternative for non 24bit-depth
        Colormap colmap = DefaultColormap(&_xDisplay, DefaultScreen(&_xDisplay));
        for (unsigned int x = 0; x < ximage.width; x++) {
            for (unsigned int y = 0; y < ximage.height; y++) {
                color.pixel = XGetPixel(&ximage, x, y);
                XQueryColor(&_xDisplay, colmap, &color);
                cv::Vec4b col = cv::Vec4b(color.blue, color.green, color.red,0);
                tmp.at<cv::Vec4b> (y,x) = col;
            }
        }
    }
    return tmp;
}


/**
 * This is the function to demonstrate the conversion cv::Mat to XImage
 * @param ximage the X image object
 * @param display the X display, for init see main() below
 * @param screen the X screen, for init see main() below
 * @return
 */
bool matToXImage(cv::Mat& im, XImage* ximage, Window& win, Display& display, Screen& screen) 
{
  if (im.empty()) return false;

    LOG_FIRST_N(INFO,1) 
      << ", width " << ximage->width 
      << ", height " << ximage->width 
      << ", offset " << ximage->xoffset 
      << ", format " << ximage->format
      << ", byte_order " << ximage->byte_order
      << ", bitmap_unit " << ximage->bitmap_unit
      << ", bitmap_pad " << ximage->bitmap_pad
      << ", depth " << ximage->depth
      << ", bytes_per_line " << ximage->depth
      << ", bits_per_pixel " << ximage->bits_per_pixel
      ;

    boost::timer t1;
    
    XColor color;
    
    LOG_FIRST_N(INFO,1) << im.cols << " " << im.rows << ", " << ximage->width << " " << ximage->height;
    //cv::Mat tmp = cv::Mat(cv::Size(ximage.width, ximage.height), CV_8UC4);

    if (screen.depths->depth == 24) {
      LOG_FIRST_N(INFO,1) << "24-bit depth";
        // Some of the following code is borrowed from http://www.roard.com/docs/cookbook/cbsu19.html ("Screen grab with X11" - by Marko Riedel, with an idea by Alexander Malmberg)
        unsigned long rmask = screen.root_visual->red_mask,
                gmask = screen.root_visual->green_mask,
                bmask = screen.root_visual->blue_mask;
        unsigned long rshift, rbits, gshift, gbits, bshift, bbits;
        //unsigned char colorChannel[3];

        rshift = 0;
        rbits = 0;
        while (!(rmask & 1)) {
            rshift++;
            rmask >>= 1;
        }
        while (rmask & 1) {
            rbits++;
            rmask >>= 1;
        }
        if (rbits > 8) {
            rshift += rbits - 8;
            rbits = 8;
        }

        gshift = 0;
        gbits = 0;
        while (!(gmask & 1)) {
            gshift++;
            gmask >>= 1;
        }
        while (gmask & 1) {
            gbits++;
            gmask >>= 1;
        }
        if (gbits > 8) {
            gshift += gbits - 8;
            gbits = 8;
        }

        bshift = 0;
        bbits = 0;
        while (!(bmask & 1)) {
            bshift++;
            bmask >>= 1;
        }
        while (bmask & 1) {
            bbits++;
            bmask >>= 1;
        }
        if (bbits > 8) {
            bshift += bbits - 8;
            bbits = 8;
        }

        LOG_FIRST_N(INFO,1) 
            << "bshift " << bshift << " bbits " << bbits 
            << ", gshift " << gshift << " gbits " << gbits 
            << ", rshift " << rshift << " rbits " << rbits;

        VLOG(4) << "matToXImage setup time " << t1.elapsed();
    
        //boost::timer t2;
        const int wd = ximage->width;
        const int ht = ximage->height;
        #if 0
        for (unsigned int y = 0; y < ht; y++) {
          for (unsigned int x = 0; x < wd; x++) {
                //colorChannel[0] = ((color.pixel >> bshift) & ((1 << bbits) - 1)) << (8 - bbits);
                //colorChannel[1] = ((color.pixel >> gshift) & ((1 << gbits) - 1)) << (8 - gbits);
                //colorChannel[2] = ((color.pixel >> rshift) & ((1 << rbits) - 1)) << (8 - rbits);
                //cv::Vec4b col = cv::Vec4b(colorChannel[0], colorChannel[1], colorChannel[0], 0);
                
                cv::Vec4b col = im.at<cv::Vec4b> (y,x);
                int b = col[0];
                int g = col[1];
                int r = col[2];
                 
                 color.pixel  = (r << rshift);
                 color.pixel |= (b << bshift);
                 color.pixel |= (g << gshift);

                if (0) {
                  XPutPixel(ximage, x,y, color.pixel);
                } else {
                  ximage->data[y * wd * 4 + x*4] = col[0];
                  ximage->data[y * wd * 4 + x*4+1] = col[1];
                  ximage->data[y * wd * 4 + x*4+2] = col[2];
                }
                //ximage->data[y * ximage->width + x*3+1] = col[1];
                //if (ht < ht/4)
                //  ximage->data[y * wd + x] = color.pixel;
                  //ximage->data[y * wd + x] = col[0];
                //ximage->data[y * ximage->width + x*3+1] = col[1];
                //ximage->data[y * ximage->width + x*3+2] = col[2];
          }
        }
        #else
         // ximage->data = (char*) im.data;
         memcpy(ximage->data, im.data, wd*ht*4); 
        #endif
        VLOG(4) << "matToXImage put pixel time " << t1.elapsed();
        LOG_FIRST_N(INFO,1) << "done copying mat";
    } else { // Extremly slow alternative for non 24bit-depth
        LOG_FIRST_N(INFO,1) <<" slow route TBD";
        Colormap colmap = DefaultColormap(&display, DefaultScreen(&display));
        for (unsigned int x = 0; x < ximage->width; x++) {
            for (unsigned int y = 0; y < ximage->height; y++) {
                color.pixel = XGetPixel(ximage, x, y);
                XQueryColor(&display, colmap, &color);
                cv::Vec4b col = cv::Vec4b(color.blue, color.green, color.red,0);
                im.at<cv::Vec4b> (y,x) = col;
            }
        }
    }
    return true;
}

}//bm
