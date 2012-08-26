
/**
 * Example program showing conversion of an image in the X11 XImage format to the openCV format IplImage
 * (by demonstrating the XImage2OpenCVImage function below)
 * The minimalistic program takes a screenshot of a specified area using XGetImage(...),
 * converts the resulting XImage to IplImage and shows the resulting openCV image until the user presses a key.
 *
 * Compile with opencv linker flags (`pkg-config --libs opencv`) e.g.
 * $ g++ ximage2opencvimage.cpp -L `pkg-config --libs opencv` -o ximage2opencvimage && ./ximage2opencvimage
 *
 * @File:   ximage2opencvimage.cpp
 * @Author: Jan Hendriks
 * @e-mail: dahoc3150@yahoo.com
 * @URL:    http://gawe-design.de
 *
 * Created on 8. March 2011, 20:01
 */

#include "screencap.h"

#include <glog/logging.h>

using namespace std;

namespace bm {

/**
 * This is the function to demonstrate the conversion XImage to IplImage
 * @param _xImage the X image object
 * @param _xDisplay the X display, for init see main() below
 * @param _xScreen the X screen, for init see main() below
 * @return
 */
cv::Mat XImage2OpenCVImage(XImage& _xImage, Display& _xDisplay, Screen& _xScreen) {
    XColor color;
    //    cout << "WxH: " << imageData->width << "x" << imageData->height << ": " << imageData->data[0] << imageData->data[1] << imageData->data[2] << imageData->format << endl;
    //IplImage* ocvImage = cvCreateImage(cv::Size(_xImage.width, _xImage.height), IPL_DEPTH_8U, 3);
    cv::Mat tmp = cv::Mat(cv::Size(_xImage.width, _xImage.height), CV_8UC3);

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

        for (unsigned int x = 0; x < _xImage.width; x++) {
            for (unsigned int y = 0; y < _xImage.height; y++) {
                color.pixel = XGetPixel(&_xImage, x, y);
                colorChannel[0] = ((color.pixel >> bshift) & ((1 << bbits) - 1)) << (8 - bbits);
                colorChannel[1] = ((color.pixel >> gshift) & ((1 << gbits) - 1)) << (8 - gbits);
                colorChannel[2] = ((color.pixel >> rshift) & ((1 << rbits) - 1)) << (8 - rbits);
                cv::Vec3b col = cv::Vec3b(colorChannel[0], colorChannel[1], colorChannel[0]);
                tmp.at<cv::Vec3b> (y,x) = col;
            }
        }
    } else { // Extremly slow alternative for non 24bit-depth
        Colormap colmap = DefaultColormap(&_xDisplay, DefaultScreen(&_xDisplay));
        for (unsigned int x = 0; x < _xImage.width; x++) {
            for (unsigned int y = 0; y < _xImage.height; y++) {
                color.pixel = XGetPixel(&_xImage, x, y);
                XQueryColor(&_xDisplay, colmap, &color);
                cv::Vec3b col = cv::Vec3b(color.blue, color.green, color.red);
                tmp.at<cv::Vec3b> (y,x) = col;
            }
        }
    }
    return tmp;
}

ScreenCap::ScreenCap( )
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
  int widthX = 640; //screen->width / 4, heightY = screen->height / 4; 
  int heightY = 480;
  
  cv::Mat out = cv::Mat(cv::Size(widthX, heightY), CV_8UC3);
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
      widthX = 640;
    }
  }

  if ((startY + heightY) > screen_h) {
    // TBD need to more intelligently cap these
    if (screen_h > heightY) {
      startY = screen_h - heightY;
    } else {
      startY = 0;
      heightY = 480;
    }
  }
 
  setSignal("startX", startX);
  setSignal("startY", startY);
  setSignal("widthX", widthX);
  setSignal("heightY", heightY);

  xImageSample = XGetImage(display, DefaultRootWindow(display), startX, startY, widthX, heightY, AllPlanes, ZPixmap);
  
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
  cv::Size sz = cv::Size(640,480);
  cv::Mat tmp1;
  cv::resize(tmp, tmp1, sz, 0, 0, cv::INTER_NEAREST );
  setImage("out", tmp);

  setDirty(); 
 
  return true;
}

} // bm

