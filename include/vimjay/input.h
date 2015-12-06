/** Copyright 2012 Lucas Walter */
#ifndef VIMJAY_INPUT_H
#define VIMJAY_INPUT_H

#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>

#include <X11/Xlib.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// #include <random>
#include <deque>
#include <map>

#include <linux/joystick.h>
#include <string>
#include <vector>

#include "vimjay/nodes.h"

namespace bm
{

// input devices will present all as a set of Signals, for instance button clicks will be 0 or 1
// and analog output will be 0-32768 or for mice the resolution of the screen

// Mouse - support multiple mice
//  also output dx, dy, mousewheel
class Mouse : public Node
{
public:
  // need to set all these - or just require an Output node pointer to be set?
  Display* display;
  Window win;
  int opcode;

  // bool run_thread;
  // boost::thread event_thread;
  // void runThread();

  explicit Mouse(const std::string name);
  ~Mouse();
  virtual void init();
  virtual bool draw(cv::Point2f ui_offset);
};

#if 0
// draw a signal into a signal buffer
class MouseSignal : public SigBuffer
{
public:
  explicit MouseSignal(const std::string name);
  virtual bool update();

  Display* display;
  Window win;
  int opcode;
};
#endif

class GamePad : public ImageNode
{
private:
  int fd;
  struct js_event js;

  std::vector<int> axis;
  std::vector<int> button;

  bool is_initted;
  bool run_thread;
  boost::thread joy_thread;

  void runThread();

public:
  explicit GamePad(const std::string name);
  ~GamePad();
  virtual void init();
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};

// Future -- wiimote?

}  // namespace bm
#endif  // VIMJAY_INPUT_H
