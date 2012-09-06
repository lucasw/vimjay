#ifndef __UTILITY_H__
#define __UTILITY_H__

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Intrinsic.h>
#include <X11/extensions/XInput2.h>

#include "opencv2/imgproc/imgproc.hpp"

namespace bm {

bool setupX(Display*& display, Window& win, const int width, const int height, int& opcode);

bool removeWindowDecorations(Display* display, Window& win); 

// TBD where is this properly defined?
typedef struct Hints{
    unsigned long   flags;
    unsigned long   functions;
    unsigned long   decorations;
    long            inputMode;
    unsigned long   status;
  } Hints;

bool matToScreen(cv::Mat& tmp, Display* display, Window& win);

cv::Mat XImage2OpenCVImage(XImage& _xImage, Display& _xDisplay, Screen& _xScreen);

bool matToXImage(cv::Mat& im, XImage* ximage, Window& win, Display& display, Screen& screen);

}

#endif

