#ifndef __SCREENCAP_H__
#define __SCREENCAP_H__

// X Server includes
#include <X11/Xlib.h>
#include <X11/Xutil.h>

// OpenCV includes
//#include <opencv/cv.h>
//#include <opencv/cvaux.h>

#include "nodes.h"

namespace bm {

class ScreenCap : public ImageNode
{
    // X resources
    Display* display;
    Screen* screen;
    XImage* xImageSample;
    IplImage* cvImageSample;
    XColor col;

  public:

  ScreenCap();
  
  virtual bool update();

    int startX;
    int startY;
    int widthX;
    int heightY;

};

} // bm

#endif //__SCREENCAP_H__

