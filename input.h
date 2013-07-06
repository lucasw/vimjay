#ifndef __INPUT_H__
#define __INPUT_H__

#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>

#include <X11/Xlib.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//#include <random>
#include <deque>
#include <map>
#include <boost/thread.hpp>

#include <linux/joystick.h>

#include "nodes.h"

namespace bm {

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
  
  //bool run_thread;
  //boost::thread event_thread;
  //void runThread();

  Mouse(const std::string name);
  ~Mouse();
  virtual void init();
  virtual bool draw(cv::Point2f ui_offset);
};

#if 0
// draw a signal into a signal buffer
class MouseSignal : public SigBuffer
{
  public: 
  MouseSignal(const std::string name);
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

  public:

  GamePad(const std::string name);
  ~GamePad();
  virtual void init();
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);

};


// Future -- wiimote?

} // bm
#endif // MISC_NODES
