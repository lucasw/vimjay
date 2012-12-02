#ifndef __UTILITY_H__
#define __UTILITY_H__

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Intrinsic.h>
#include <X11/extensions/XInput2.h>

#include "opencv2/imgproc/imgproc.hpp"

#include "nodes.h"

// bash color codes
#define CLNRM "\e[0m"
#define CLWRN "\e[0;43m"
#define CLERR "\e[1;41m"
#define CLVAL "\e[1;36m"
#define CLTXT "\e[1;35m"
// BOLD black text with blue background
#define CLTX2 "\e[1;44m"  

#define MAT_FORMAT_C3 CV_8UC4
#define MAT_FORMAT CV_8U
//#define MAT_FORMAT_C3 CV_16SC3
//#define MAT_FORMAT CV_16S
//#define MAT_FORMAT_C3 CV_32FC3
//#define MAT_FORMAT CV_32F

namespace bm {

class Node;

std::string getId(Node* ptr);

std::string logMat(const cv::Mat& m);

bool getBezier(
      const std::vector<cv::Point2f>& control_points, // TBD currently has to be 4
      std::vector<cv::Point2f>& output_points,
      const int num // numbe of intermediate points to generate 
      );
 
Window get_toplevel_parent(Display * display, Window window);

bool setupX(Display*& display, Window& win, const int width, const int height, int& opcode);

bool setWindowDecorations(Display* display, Window& win, bool decorations_on); 

// TBD where is this properly defined?
typedef struct Hints{
    unsigned long   flags;
    unsigned long   functions;
    unsigned long   decorations;
    long            inputMode;
    unsigned long   status;
  } Hints;

bool matToScreen(cv::Mat& tmp, Display* display, Window& win);

cv::Mat XImage2OpenCVImage(XImage& ximage, Display& _xDisplay, Screen& _xScreen);

bool matToXImage(cv::Mat& im, XImage* ximage, Window& win, Display& display, Screen& screen);

}

#endif

