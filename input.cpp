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

#include "input.h"

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <typeinfo>
#include <cxxabi.h> // non portable

#include <deque>
//#include <pair>


#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "input.h"
#include "nodes.h" 
#include "misc_nodes.h" 
#include "signals.h"
#include "filter.h"
#include "generate.h"
#include "screencap.h"
#include "utility.h"

//using namespace cv;
using namespace std;

namespace bm {
//DEFINE_int32(width, 640, "");
//DEFINE_int32(height, 480, "");
//DEFINE_string(mouse, "/dev/input/mouse0", "/dev/input/mouseN or /dev/input/eventN");
/*
 * To work with Kinect the user must install OpenNI library and PrimeSensorModule for OpenNI and
 * configure OpenCV with WITH_OPENNI flag is ON (using CMake).

TBD have a mode that takes a webcam, uses brightness as depth, and thresholds it for the valid map

 */
#if 0
int main( int argc, char* argv[] )
{
 // google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  google::ParseCommandLineFlags(&argc, &argv, false);
#endif
#if 0
  int fd;
  // cat /proc/bus/input/devices - TBD how to find just the mouses?
  // - TBD how to find just the mouses?
  // TBD can't get touchpad to work, don't even see it when catting /dev/input/mouseN
  if ((fd = open(FLAGS_mouse.c_str(), O_RDONLY)) < 0) {
    LOG(ERROR) << "couldn't open mouse " << fd;
    exit(0);
  }
  struct input_event ev;

  bool rv = true;
  while(rv) {

    read(fd, &ev, sizeof(struct input_event));
    VLOG(1) << "value 0x" << std::hex << ev.value 
      << ", type 0x" << std::hex << ev.type 
      << ", code 0x" << std::hex << ev.code;
    if (ev.type == EV_REL) {
      if (ev.value != 0) {
        // mouse move left
        if (ev.code == ABS_X) LOG(INFO)<< "dx " << ev.value;
        // mouse move right
        if (ev.code == ABS_Y)  LOG(INFO)<< "dy " << ev.value;
        // wheel
        if (ev.code == REL_WHEEL) LOG(INFO)<< "wheel " << ev.value;
      }
    }
    if (ev.type == EV_MSC) {
      // 0x90001 - 3
      LOG(INFO) << "Button value 0x" << std::hex << ev.value 
      << ", code " << ev.code;

    }
  }
#endif

#if 0
  int width = FLAGS_width;
  int height = FLAGS_height;
  
	Display *display;
  Window win;
  int opcode;
  bm::setupX(display, win, width, height, opcode);
  //bm::setWindowDecorations(display, win,false);

  cv::Mat tmp;
  // BGR
  tmp = cv::Mat(height, width, CV_8UC4, cv::Scalar(255,100,50));
 
  bm::matToScreen(tmp, display, win);

  //cv::imshow("temp", tmp);
  //cv::waitKey(10);

	/* Event loop */
	while(1) {

    bm::matToScreen(tmp, display, win);
    usleep(10000);
#endif
    
Mouse::Mouse(const std::string name) : 
  Node(name),
  display(NULL) 
{
}

void Mouse::init()
{
  Node::init();
  setSignal("0_x", 0);

  //event_thread = boost::thread(&Mouse::runThread, this);
}

Mouse::~Mouse()
{
  //run_thread = false;
  //event_thread.join();
}
#if 0
bool Mouse::update()
{
  if (!Node::update()) return false;

  return true;
}
#endif

bool Mouse::draw(cv::Point2f ui_offset)
{
  Node::draw(ui_offset);
  //run_thread = true;
 
  vector<string> sig_name;
  vector<float> sig_val;

  /// TBD it would be nice to be able to get the display
  /// of the root window so all mouse moves can be in here
  /// or optionally the display of the preview window, or 
  /// gui window
  if (!getMouse(display, opcode, sig_name, sig_val))
    return false;      
 
  for (size_t i = 0; i < sig_name.size(); i++) {
    setSignal(sig_name[i], sig_val[i]);
  }
 
  return true;
}

#if 0
MouseSignal::MouseSignal(const std::string name) : 
  SigBuffer(name),
  display(NULL) 
{ 
  sigs.resize( Config::inst()->im_width);
}

bool MouseSignal::draw( )
{
  
  vector<string> sig_name;
  vector<float> sig_val;
  // TBD getMouse is completely unsuitable for this
  if (!getMouse(display, opcode, sig_name, sig_val))
    return false;      

  
 
  return true;
}
#endif


} //  bm

