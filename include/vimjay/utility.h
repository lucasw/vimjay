/** Copyright 2012 Lucas Walter */
#ifndef VIMJAY_UTILITY_H
#define VIMJAY_UTILITY_H

#if 0
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Intrinsic.h>
#include <X11/extensions/XInput2.h>
#endif

#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <utility>
#include <vector>

#include "vimjay/nodes.h"

#if 0
// bash color codes, screw up ros console output
#define CLNRM "\e[0m"
#define CLWRN "\e[0;43m"
#define CLERR "\e[1;41m"
#define CLVAL "\e[1;36m"
#define CLTXT "\e[1;35m"
// BOLD black text with blue background
#define CLTX2 "\e[1;44m"
#else
#define CLNRM ""
#define CLWRN ""
#define CLERR ""
#define CLVAL ""
#define CLTXT ""
// BOLD black text with blue background
#define CLTX2 ""
#endif

#define MAT_FORMAT_C3 CV_8UC4
#define MAT_FORMAT CV_8U
// #define MAT_FORMAT_C3 CV_16SC3
// #define MAT_FORMAT CV_16S
// #define MAT_FORMAT_C3 CV_32FC3
// #define MAT_FORMAT CV_32F

namespace bm
{

class Node;

bool getImageNamesAndSubDirs(const std::string dir,
    std::vector<std::string>& image_names, std::vector<std::string>& sub_dirs);

void initRemaps(cv::Mat& base_x, cv::Mat& base_y);

std::string getId(boost::shared_ptr<Node> ptr);

std::string logMat(const cv::Mat& m);

bool getBezier(
  const std::vector<cv::Point2f>& control_points, // TBD currently has to be 4
  std::vector<cv::Point2f>& output_points,
  // numbe of intermediate points to generate
  const int num);

#if 0
bool get_toplevel_parent(
  Display* display,
  Window window,
  Window& cur_window);

bool setupX(Display*& display, Window& win, const int width, const int height, int& opcode);

bool setWindowDecorations(Display* display, Window& win, bool decorations_on);

// TBD where is this properly defined?
typedef struct Hints
{
  unsigned long   flags;
  unsigned long   functions;
  unsigned long   decorations;
  long            inputMode;
  unsigned long   status;
}
Hints;

bool matToScreen(cv::Mat& tmp, Display* display, Window& win);

cv::Mat XImage2OpenCVImage(XImage& ximage, Display& _xDisplay, Screen& _xScreen);

bool matToXImage(cv::Mat& im, XImage* ximage, Window& win, Display& display, Screen& screen);

bool getEv(
  Display* display,
  XEvent& ev,
  std::vector<std::string>& sig_name,
  std::vector<float>& sig_val,
  std::vector< std::pair<char, bool> >& keys);

bool getMouse(
  Display* display,
  const int opcode,
  std::vector<std::string>& sig_name,
  std::vector<float>& sig_val,
  std::vector<std::pair<char, bool> >& keys);
#endif

bool fixAspect(cv::Mat& tmp0, cv::Mat& tmp1, const int mode);
bool fixAspectFill(cv::Mat& tmp0, cv::Mat& tmp1, const int mode);

bool getVideoFrame(
  cv::VideoCapture& video,
  cv::Mat& dst,
  const std::string name,
  const int mode_type,
  const int aspect_mode
);

}  // namespace bm

#endif  // VIMJAY_UTILITY

