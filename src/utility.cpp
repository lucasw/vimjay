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



#include "vimjay/utility.h"

#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>

#include "vimjay/config.h"

#include <boost/timer/timer.hpp>
#include <ros/console.h>

#include <typeinfo>
#include <cxxabi.h> // non portable

namespace bm
{


/**
  Provide a list of all the images (png or jpg) in a directory,
  and also all the subdirectories.

  TBD run this in separate thread
*/
bool getImageNamesAndSubDirs(const std::string dir, std::vector<std::string>& image_names, std::vector<std::string>& sub_dirs)
{

  image_names.clear();
  sub_dirs.clear();

  ROS_DEBUG_STREAM_COND(log_level > 1, "get image names " << dir);

  boost::filesystem::path image_path(dir);
  if (!is_directory(image_path))
  {
    ROS_ERROR_STREAM(CLERR << " not a directory " << CLNRM << dir);
    return false;
  }

  boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end


  boost::filesystem::directory_iterator itr;

  try
  {
    itr = boost::filesystem::directory_iterator(image_path);
  }
  // catch (boost::exception const& ex)
  catch (const boost::filesystem::filesystem_error& ex)
  {
    ROS_ERROR_STREAM(dir << ": " << boost::diagnostic_information(ex));
    return false;
  }

  // for ( boost::filesystem::directory_iterator itr( image_path );
  //    itr != end_itr;
  //    ++itr )
  while (itr != end_itr)
  {

    std::stringstream ss;
    ss << *itr;
    std::string str = (ss.str());
    // strip off "" at beginning/end
    str = str.substr(1, str.size() - 2);


    if (is_directory(*itr))
    {
      sub_dirs.push_back(str);
      ROS_DEBUG_STREAM_COND(log_level > 1, dir << " sub dir " << str);
    }
    else
    {
      if (!((str.substr(str.size() - 3, 3) != "jpg") ||
            (str.substr(str.size() - 3, 3) != "png"))
         )
      {
        ROS_DEBUG_STREAM_COND(log_level > 1, "not expected image type: " << str);
        continue;
      }

      image_names.push_back(str);
      ROS_DEBUG_STREAM_COND(log_level > 1, dir << " image " << str);
    }

    ++itr;
  } //

  return true;
} // get image and sub dir names

void initRemaps(cv::Mat& base_x, cv::Mat& base_y)
{
  base_x = cv::Mat(Config::inst()->getImSize(), CV_32FC1);
  base_y = base_x.clone();

  for (int i = 0; i < base_x.rows; i++)
  {
    for (int j = 0; j < base_x.cols; j++)
    {
      base_x.at<float>(i, j) = j;
      base_y.at<float>(i, j) = i;
    }
  }
}

std::string getId(boost::shared_ptr<Node> ptr)
{
  int status;
  return (abi::__cxa_demangle(typeid(*ptr).name(), 0, 0, &status));
}

std::string logMat(const cv::Mat& m)
{
  std::stringstream s;
  s << "{";
  for (int x = 0; x < m.rows; x++)
  {
    for (int y = 0; y < m.cols; y++)
    {
      s << " " << m.at<double>(x, y);
    }
    if (x < m.rows - 1) s << std::endl;
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
  if (control_points.size() != 4)
  {
    ROS_ERROR_STREAM(control_points.size() << " != 4");
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
  double coeff_raw[4][4] =
  {
    { 1, 0, 0, 0},
    { -3, 3, 0, 0},
    { 3, -6, 3, 0},
    { -1, 3, -3, 1}
  };
  cv::Mat coeff = cv::Mat(4, 4, CV_64F, coeff_raw);
  cv::Mat control = cv::Mat::zeros(4, 2, CV_64F);

  for (int i = 0; i < control.rows; i++)
  {
    control.at<double>(i, 0) = control_points[i].x;
    control.at<double>(i, 1) = control_points[i].y;
  }

  ROS_DEBUG_STREAM_COND(log_level > 5, CLTXT << "coeff " << CLNRM << std::endl << logMat(coeff));
  ROS_DEBUG_STREAM_COND(log_level > 5, CLTXT << "control " << CLNRM << std::endl << logMat(control));

  cv::Point2f old_pt;

  output_points.clear();

  for (int i = 0; i < num; i++)
  {
    float t = (float)i / (float)(num - 1);

    // concentrate samples near beginning and end
    if (t < 0.5)
    {
      t *= t;
    }
    else
    {
      t = 1.0 - (1.0 - t) * (1.0 - t);
    }
    double tee_raw[1][4] = {{ 1.0, t, t * t, t*t * t}};

    cv::Mat tee = cv::Mat(1, 4, CV_64F, tee_raw);
    cv::Mat pos = tee * coeff * control;

    cv::Point new_pt = cv::Point2f(pos.at<double>(0, 0), pos.at<double>(0, 1));

    output_points.push_back(new_pt);

    ROS_DEBUG_STREAM_COND(log_level > 5, "pos " << t << " "
                          << new_pt.x << " " << new_pt.y
                          << std::endl << logMat(tee)
                          << std::endl << logMat(pos));
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////
// XWindows
#if 0

/*
   Returns the parent window of "window" (i.e. the ancestor of window
   that is a direct child of the root, or window itself if it is a direct child).
   If window is the root window, returns window.

   From http://stackoverflow.com/questions/3909713/xlib-xgetwindowattributes-always-returns-1x1
   */
bool get_toplevel_parent(
  Display* display,
  Window window,
  Window& cur_window
)
{
  Window parent;
  Window root;
  Window * children;
  unsigned int num_children;

  cur_window = window;

  int i = 0;
  while (1)
  {
    ROS_DEBUG_STREAM_COND(log_level > 2, "x query tree " << i++ << " "
                          << display << " "
                          << cur_window << " ");
    if (0 == XQueryTree(display, cur_window, &root,
                        &parent, &children, &num_children))
    {
      ROS_ERROR_STREAM("XQueryTree error");
      return false;
      // abort(); //change to whatever error handling you prefer
    }
    if (children)   // must test for null
    {
      XFree(children);
    }
    if (cur_window == root)   // || parent == root) {
    {
      ROS_INFO_STREAM(window << " " << CLVAL << cur_window << CLNRM
                      << " " << root);
      return true;
    }
    else
    {
      cur_window = parent;
    }
  }
  return false;
}

bool setupX(Display*& display, Window& win, const int width, const int height, int& opcode)
{
  // from git://gitorious.org/vinput/vinput.git demo-paint.c (GPL)
  /* Connect to the X server */
  display = XOpenDisplay(NULL);


  /* Check if the XInput Extension is available */
  int event, error;
  if (!XQueryExtension(display, "XInputExtension", &opcode, &event, &error))
  {
    printf("X Input extension not available.\n");
    return false;
  }
  ROS_INFO_STREAM("XInputExtension available");

  /* Check for XI2 support */
  int major = 2, minor = 0;
  if (XIQueryVersion(display, &major, &minor) == BadRequest)
  {
    printf("XI2 not available. Server supports %d.%d\n", major, minor);
    return false;
  }
  ROS_INFO_STREAM("XI2 available");

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
  // TBD make optional?
  XISetMask(mask, XI_KeyPress);
  XISetMask(mask, XI_KeyRelease);

  /* Register events on the window */
  int screen_num = DefaultScreen(display);
  win = XCreateSimpleWindow(display, DefaultRootWindow(display),
                            0, 0, width, height, 5,
                            WhitePixel(display, screen_num),
                            BlackPixel(display, screen_num));
  XMapWindow(display, win);
  XSync(display, False);

  XISelectEvents(display, win, &eventmask, 1);

  if (0)
  {
    // move to upper corner?  No it moves it to the far left but a few menu bar widths below the top of the screen
    Window wid = DefaultRootWindow(display);
    XMoveResizeWindow(display, wid, 0, 0, width, height);
    XMapRaised(display, wid);
  }

  {
    Window wid = DefaultRootWindow(display);
    XWindowAttributes xwAttr;
    Status ret = XGetWindowAttributes(display, wid, &xwAttr);
    int screen_w = xwAttr.width;
    int screen_h = xwAttr.height;

    ROS_INFO_STREAM(" screen w h " << CLVAL << screen_w << " " << screen_h << CLNRM);
    // This is the same as the attributes above
    ROS_INFO_STREAM(XDisplayWidth(display, 0) << " " << XDisplayHeight(display, 0));
    // this is junk
    ROS_INFO_STREAM(XDisplayWidth(display, 1) << " " << XDisplayHeight(display, 1));

    // TBD how to get resolution of particular monitor?

  }

  return true;
}

bool setWindowDecorations(Display* display, Window& win, bool decorations_on)
{
  ROS_DEBUG_STREAM_COND(log_level > 2, "setting window decorations " << decorations_on << " " << win);
  // code to remove decoration
  Hints hints;
  Atom property;
  hints.flags = 2;
  hints.decorations = decorations_on ? 1 : 0;
  property = XInternAtom(display, "_MOTIF_WM_HINTS", true);
  XChangeProperty(display, win, property, property, 32, PropModeReplace, (unsigned char *)&hints, 5);
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
  // XImage* ximage = XCreateImage(display, DefaultVisual(display, screen)
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
  // IplImage* ocvImage = cvCreateImage(cv::Size(ximage.width, ximage.height), IPL_DEPTH_8U, 3);
  cv::Mat tmp = cv::Mat(cv::Size(ximage.width, ximage.height), CV_8UC4);

  if (_xScreen.depths->depth == 24)
  {
    // Some of the following code is borrowed from http://www.roard.com/docs/cookbook/cbsu19.html ("Screen grab with X11" - by Marko Riedel, with an idea by Alexander Malmberg)
    unsigned long rmask = _xScreen.root_visual->red_mask,
                  gmask = _xScreen.root_visual->green_mask,
                  bmask = _xScreen.root_visual->blue_mask;
    unsigned long rshift, rbits, gshift, gbits, bshift, bbits;
    unsigned char colorChannel[3];

    rshift = 0;
    rbits = 0;
    while (!(rmask & 1))
    {
      rshift++;
      rmask >>= 1;
    }
    while (rmask & 1)
    {
      rbits++;
      rmask >>= 1;
    }
    if (rbits > 8)
    {
      rshift += rbits - 8;
      rbits = 8;
    }

    gshift = 0;
    gbits = 0;
    while (!(gmask & 1))
    {
      gshift++;
      gmask >>= 1;
    }
    while (gmask & 1)
    {
      gbits++;
      gmask >>= 1;
    }
    if (gbits > 8)
    {
      gshift += gbits - 8;
      gbits = 8;
    }

    bshift = 0;
    bbits = 0;
    while (!(bmask & 1))
    {
      bshift++;
      bmask >>= 1;
    }
    while (bmask & 1)
    {
      bbits++;
      bmask >>= 1;
    }
    if (bbits > 8)
    {
      bshift += bbits - 8;
      bbits = 8;
    }

    const int wd = ximage.width;
    const int ht = ximage.height;
    memcpy(tmp.data, ximage.data, wd * ht * 4);
#if 0
    for (unsigned int x = 0; x < ximage.width; x++)
    {
      for (unsigned int y = 0; y < ximage.height; y++)
      {
        color.pixel = XGetPixel(&ximage, x, y);
        colorChannel[0] = ((color.pixel >> bshift) & ((1 << bbits) - 1)) << (8 - bbits);
        colorChannel[1] = ((color.pixel >> gshift) & ((1 << gbits) - 1)) << (8 - gbits);
        colorChannel[2] = ((color.pixel >> rshift) & ((1 << rbits) - 1)) << (8 - rbits);
        cv::Vec4b col = cv::Vec4b(colorChannel[0], colorChannel[1], colorChannel[0], 0);
        tmp.at<cv::Vec4b> (y, x) = col;
      }
    }
#endif
  }
  else     // Extremly slow alternative for non 24bit-depth
  {
    Colormap colmap = DefaultColormap(&_xDisplay, DefaultScreen(&_xDisplay));
    for (unsigned int x = 0; x < ximage.width; x++)
    {
      for (unsigned int y = 0; y < ximage.height; y++)
      {
        color.pixel = XGetPixel(&ximage, x, y);
        XQueryColor(&_xDisplay, colmap, &color);
        cv::Vec4b col = cv::Vec4b(color.blue, color.green, color.red, 0);
        tmp.at<cv::Vec4b> (y, x) = col;
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

  ROS_INFO_STREAM_ONCE("mat to ximage "
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
                      );

  boost::timer t1;

  XColor color;

  ROS_INFO_STREAM_ONCE("image size " << im.cols << " " << im.rows << ", "
                       << ximage->width << " " << ximage->height);
  // cv::Mat tmp = cv::Mat(cv::Size(ximage.width, ximage.height), CV_8UC4);

  if (screen.depths->depth == 24)
  {
    ROS_INFO_ONCE("24-bit depth");
    // Some of the following code is borrowed from http://www.roard.com/docs/cookbook/cbsu19.html ("Screen grab with X11" - by Marko Riedel, with an idea by Alexander Malmberg)
    unsigned long rmask = screen.root_visual->red_mask,
                  gmask = screen.root_visual->green_mask,
                  bmask = screen.root_visual->blue_mask;
    unsigned long rshift, rbits, gshift, gbits, bshift, bbits;
    // unsigned char colorChannel[3];

    rshift = 0;
    rbits = 0;
    while (!(rmask & 1))
    {
      rshift++;
      rmask >>= 1;
    }
    while (rmask & 1)
    {
      rbits++;
      rmask >>= 1;
    }
    if (rbits > 8)
    {
      rshift += rbits - 8;
      rbits = 8;
    }

    gshift = 0;
    gbits = 0;
    while (!(gmask & 1))
    {
      gshift++;
      gmask >>= 1;
    }
    while (gmask & 1)
    {
      gbits++;
      gmask >>= 1;
    }
    if (gbits > 8)
    {
      gshift += gbits - 8;
      gbits = 8;
    }

    bshift = 0;
    bbits = 0;
    while (!(bmask & 1))
    {
      bshift++;
      bmask >>= 1;
    }
    while (bmask & 1)
    {
      bbits++;
      bmask >>= 1;
    }
    if (bbits > 8)
    {
      bshift += bbits - 8;
      bbits = 8;
    }

    ROS_INFO_STREAM_ONCE("im "
                         << "bshift " << bshift << " bbits " << bbits
                         << ", gshift " << gshift << " gbits " << gbits
                         << ", rshift " << rshift << " rbits " << rbits
                        );

    ROS_DEBUG_STREAM_COND(log_level > 4, "matToXImage setup time " << t1.format());

    // boost::timer t2;
    const int wd = ximage->width;
    const int ht = ximage->height;
#if 0
    for (unsigned int y = 0; y < ht; y++)
    {
      for (unsigned int x = 0; x < wd; x++)
      {
        // colorChannel[0] = ((color.pixel >> bshift) & ((1 << bbits) - 1)) << (8 - bbits);
        // colorChannel[1] = ((color.pixel >> gshift) & ((1 << gbits) - 1)) << (8 - gbits);
        // colorChannel[2] = ((color.pixel >> rshift) & ((1 << rbits) - 1)) << (8 - rbits);
        // cv::Vec4b col = cv::Vec4b(colorChannel[0], colorChannel[1], colorChannel[0], 0);

        cv::Vec4b col = im.at<cv::Vec4b> (y, x);
        int b = col[0];
        int g = col[1];
        int r = col[2];

        color.pixel  = (r << rshift);
        color.pixel |= (b << bshift);
        color.pixel |= (g << gshift);

        if (0)
        {
          XPutPixel(ximage, x, y, color.pixel);
        }
        else
        {
          ximage->data[y * wd * 4 + x * 4] = col[0];
          ximage->data[y * wd * 4 + x * 4 + 1] = col[1];
          ximage->data[y * wd * 4 + x * 4 + 2] = col[2];
        }
        // ximage->data[y * ximage->width + x*3+1] = col[1];
        // if (ht < ht/4)
        //  ximage->data[y * wd + x] = color.pixel;
        // ximage->data[y * wd + x] = col[0];
        // ximage->data[y * ximage->width + x*3+1] = col[1];
        // ximage->data[y * ximage->width + x*3+2] = col[2];
      }
    }
#else
    // ximage->data = (char*) im.data;
    memcpy(ximage->data, im.data, wd * ht * 4);
#endif
    ROS_DEBUG_STREAM_COND(log_level > 4, "matToXImage put pixel time " << t1.format());
    ROS_INFO_ONCE("done copying mat");
  }
  else     // Extremly slow alternative for non 24bit-depth
  {
    ROS_INFO_ONCE(" slow route TBD");
    Colormap colmap = DefaultColormap(&display, DefaultScreen(&display));
    for (unsigned int x = 0; x < ximage->width; x++)
    {
      for (unsigned int y = 0; y < ximage->height; y++)
      {
        color.pixel = XGetPixel(ximage, x, y);
        XQueryColor(&display, colmap, &color);
        cv::Vec4b col = cv::Vec4b(color.blue, color.green, color.red, 0);
        im.at<cv::Vec4b> (y, x) = col;
      }
    }
  }
  return true;
}

bool getEv(
  Display* display,
  XEvent& ev,
  std::vector<std::string>& sig_name,
  std::vector<float>& sig_val,
  std::vector< std::pair<char, bool> >& keys
)
{
  XIDeviceEvent* evData = (XIDeviceEvent*)(ev.xcookie.data);
  const int deviceid = evData->deviceid;

  XKeyPressedEvent key_data;

  switch (ev.xcookie.evtype)
  {
  case XI_Motion:
    ROS_DEBUG_STREAM_COND(log_level > 1, "motion " << deviceid << " " << evData->event_x
                          << " " << evData->event_y);
    sig_name.push_back(boost::lexical_cast<std::string>(deviceid) + "_x");
    sig_val.push_back(evData->event_x);
    sig_name.push_back(boost::lexical_cast<std::string>(deviceid) + "_y");
    sig_val.push_back(evData->event_y);
    break;

  case XI_ButtonPress:
    ROS_DEBUG_STREAM_COND(log_level > 1, deviceid << " button: " << evData->detail);
    sig_name.push_back(boost::lexical_cast<std::string>(deviceid) + "_" +
                       boost::lexical_cast<std::string>(evData->detail));
    sig_val.push_back(1);
    break;

  case XI_ButtonRelease:
    ROS_DEBUG_STREAM_COND(log_level > 1, deviceid << " unclick");
    sig_name.push_back(boost::lexical_cast<std::string>(deviceid) + "_" +
                       boost::lexical_cast<std::string>(evData->detail));
    sig_val.push_back(0);
    break;

  case XI_KeyPress:
  case XI_KeyRelease:
    // ROS_INFO_STREAM("key down");

    // Assign info from our XIDeviceEvent to a standard XKeyPressedEvent which
    // XLookupString() can actually understand:
    key_data.type         = KeyPress;
    key_data.root         = evData->root;
    key_data.window       = evData->event;
    key_data.subwindow    = evData->child;
    key_data.time         = evData->time;
    key_data.x            = evData->event_x;
    key_data.y            = evData->event_y;
    key_data.x_root       = evData->root_x;
    key_data.y_root       = evData->root_y;
    key_data.same_screen  = True;
    key_data.send_event   = False;
    key_data.serial       = evData->serial;
    key_data.display      = display;

    key_data.keycode      = evData->detail;
    key_data.state        = evData->mods.effective;

    char asciiChar;
    if (1 == XLookupString(((XKeyEvent*)(&key_data)), &asciiChar, 1, NULL, NULL))
    {
      // Mapped: Assign it.
      const bool key_down = (ev.xcookie.evtype == XI_KeyPress);

      keys.push_back(std::pair<char, bool>(asciiChar, key_down));
      ROS_DEBUG_STREAM_COND(log_level > 2, deviceid << "key " << (int)asciiChar << " "
                            << asciiChar << " "
                            << key_down);
    }

    break;


  } // switch

  return true;
}

/// TBD just make thread hang out and continually poll mouse,
/// and have singleton object provide current mouse position within
/// root or any window, and whether button is clicked or not.
//
/// get mouse device and x and y position and button clicks
bool getMouse(
  Display* display,
  const int opcode,
  std::vector<std::string>& sig_name,
  std::vector<float>& sig_val,
  std::vector< std::pair<char, bool> >& keys
)
{
  if (!display)
  {
    ROS_ERROR_STREAM("no display");
    return false;
  }
  // ROS_DEBUG_STREAM_COND(log_level > 1, "mouse");

  XEvent ev;
  /* Get next event; blocks until an event occurs */
  while (XPending(display))
  {
    XNextEvent(display, &ev);
    if (ev.xcookie.type == GenericEvent &&
        ev.xcookie.extension == opcode &&
        XGetEventData(display, &ev.xcookie))
      // if (XCheckWindowEvent(display, win, PointerMotionMask | ButtonPressMask | ButtonReleaseMask, &ev))
    {
      ROS_DEBUG_STREAM_COND(log_level > 1, " event found");
      getEv(display, ev, sig_name, sig_val, keys);
    } // correct event

    XFreeEventData(display, &ev.xcookie);

  } // while
  // usleep(1000);
  return true;
} // getMouse

#endif

/// resize the source tmp0 mat to fit inside tmp1 with borders
/// tmp0 and tmp1 have to be initialized already
/// TBD add another mode which chops off the edges so there
/// are no borders?
bool fixAspect(cv::Mat& tmp0, cv::Mat& tmp1, const int mode)
{
  const float aspect_0 = (float)tmp0.cols / (float)tmp0.rows;
  const float aspect_1 = (float)tmp1.cols / (float)tmp1.rows;

  const cv::Size sz = tmp1.size();

  // this is the subimage that has to fit within tmp1
  // it will be shrunk down as necessary and border offset
  // values adjusted
  cv::Size tmp_sz = tmp1.size();
  int off_x = 0;
  int off_y = 0;

  // TBD could have epsilon defined by 1 pixel width
  if (aspect_0 > aspect_1)
  {
    // have to have a border on top
    tmp_sz.height = tmp_sz.width / aspect_0;
    off_y = (sz.height - tmp_sz.height) / 2;
  }
  else if (aspect_0 < aspect_1)
  {
    // have a border on the sides
    tmp_sz.width = tmp_sz.height * aspect_0;
    off_x = (sz.width - tmp_sz.width) / 2;
  }

  ROS_DEBUG_STREAM_COND(log_level > 2, "fix aspect " << aspect_0 << " " << aspect_1 << ", "
                        << off_x << " " << off_y << ", "
                        << tmp_sz.width << " " << tmp_sz.height << ", "
                        << sz.width << " " << sz.height);

  // the source image with the right aspect ratio and size
  // to fit within the dest image
  cv::Mat tmp_aspect;
  cv::resize(tmp0, tmp_aspect, tmp_sz, 0, 0, mode);

  // TBD put offset so image is centered
  cv::Mat tmp1_roi = tmp1(cv::Rect(off_x, off_y, tmp_sz.width, tmp_sz.height));
  tmp_aspect.copyTo(tmp1_roi);


  return true;
}

/// another mode which chops off the edges so there
/// are no borders
bool fixAspectFill(cv::Mat& tmp0, cv::Mat& tmp1, const int mode)
{
  // width/height
  const float aspect_0 = (float)tmp0.cols / (float)tmp0.rows;
  const float aspect_1 = (float)tmp1.cols / (float)tmp1.rows;

  const cv::Size src_sz = tmp0.size();
  const cv::Size sz = tmp1.size();

  // this is the subimage that has to fit within tmp1
  // it will be shrunk down as necessary and border offset
  // values adjusted
  cv::Size tmp_sz = sz;
  int off_x = 0;
  int off_y = 0;

  // TBD could have epsilon defined by 1 pixel width
  if (aspect_0 > aspect_1)
  {
    // lose the edges off the sides
    tmp_sz.width = sz.height * aspect_0;
    off_x = (tmp_sz.width - sz.width) / 2;
  }
  else if (aspect_0 < aspect_1)
  {
    // lose the edges of the top and bottom
    tmp_sz.height = sz.width / aspect_0;
    off_y = (tmp_sz.height - sz.height) / 2;
  }

  ROS_DEBUG_STREAM_COND(log_level > 2, "fix aspect fill "
                        << "src " << src_sz.width << " " << src_sz.height << ", "
                        << "tmp " << off_x << " " << off_y << " "
                        << tmp_sz.width << " " << tmp_sz.height << ", "
                        << "dst " << sz.width << " " << sz.height);

  // resize the source image so that it is equal to the dest
  // image in at least one dimension,
  cv::Mat tmp_aspect;
  cv::resize(tmp0, tmp_aspect, tmp_sz, 0, 0, mode);

  // now take just the subimage so the aspect is preserved
  // while the resolution is dst
  cv::Mat tmp1_roi = tmp_aspect(cv::Rect(off_x, off_y, sz.width, sz.height));
  tmp1_roi.copyTo(tmp1);

  return true;
}

bool getVideoFrame(
  cv::VideoCapture& video,
  cv::Mat& dst,
  const std::string name,
  const int mode_type,
  const int aspect_mode
)
{
  if (!video.grab())
  {
    // TBD this is only an error with a live webcam
    // ROS_ERROR_STREAM(name << " Can not grab images." << endl);
    // error_count++;
    return false;
  }

  cv::Mat new_out;
  video.retrieve(new_out);

  if (new_out.empty())
  {
    ROS_ERROR_STREAM(name << " new image empty");
    // error_count++;
    return false;
  }

  // error_count--;
  // if (error_count < 0) error_count = 0;
  // I think opencv is reusing a mat within video so have to clone it

  // if (&new_out.data == &out.data) {
  //
  // cv::Mat tmp; // new_out.clone();

  float scale = 1.0;
  if (MAT_FORMAT == CV_16S) scale = 255;
  if (MAT_FORMAT == CV_32F) scale = 1.0 / 255.0;
  if (MAT_FORMAT == CV_8U) scale = 1.0;
  cv::Mat tmp0 = cv::Mat(new_out.size(), CV_8UC4, cv::Scalar(0));
  // just calling reshape(4) doesn't do the channel reassignment like this does
  int ch[] = {0, 0, 1, 1, 2, 2};
  mixChannels(&new_out, 1, &tmp0, 1, ch, 3);
  cv::Mat tmp;
  tmp0.convertTo(tmp, MAT_FORMAT, scale); //, 1.0/(255.0));//*255.0*255.0*255.0));

  if (aspect_mode == 1)
  {
    dst = cv::Mat(Config::inst()->getImSize(), MAT_FORMAT_C3);
    fixAspect(tmp, dst, mode_type);
  }
  else if (aspect_mode == 2)
  {
    dst = cv::Mat(Config::inst()->getImSize(), MAT_FORMAT_C3);
    fixAspectFill(tmp, dst, mode_type);
  }
  else
  {
    // resize will create the dst Mat properly
    const cv::Size sz = Config::inst()->getImSize();
    cv::resize(tmp, dst, sz, 0, 0, mode_type);
  }

  return true;
}


} // bm
