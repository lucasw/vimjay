#ifndef __UTILITY_H__
#define __UTILITY_H__

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Intrinsic.h>
#include <X11/extensions/XInput2.h>

#include "opencv2/imgproc/imgproc.hpp"

namespace bm {

cv::Mat XImage2OpenCVImage(XImage& _xImage, Display& _xDisplay, Screen& _xScreen);

bool matToXImage(cv::Mat& im, XImage* ximage, Window& win, Display& display, Screen& screen);

}

#endif

