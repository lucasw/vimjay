/*
  
  Copyright 2012 Lucas Walter

     This file is part of Camthing.

    Camthing is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Camthing is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Camthing.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "camthing.h"

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <typeinfo>
#include <cxxabi.h> // non portable

#include <deque>
//#include <pair>

#include <boost/lexical_cast.hpp>

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
  tmp = cv::Mat(height, width, CV_8UC3, cv::Scalar(255,100,50));
 
  bm::matToScreen(tmp, display, win);

  //cv::imshow("temp", tmp);
  //cv::waitKey(10);

	/* Event loop */
	while(1) {

    bm::matToScreen(tmp, display, win);
    usleep(10000);
#endif
    

bool Mouse::update()
    //bool getMouse(Display* display, int& deviceid, int& button, int& x, int& y, bool );
    {
      if (!Node::update()) return false;

      if (!display) return true;
      VLOG(1) << "mouse";

      XEvent ev;
      /* Get next event; blocks until an event occurs */
      // TBD block is bad, move to another thread
      XNextEvent(display, &ev);
      if (ev.xcookie.type == GenericEvent &&
          ev.xcookie.extension == opcode &&
          XGetEventData(display, &ev.xcookie))
      {
        XIDeviceEvent* evData = (XIDeviceEvent*)(ev.xcookie.data);
        int deviceid = evData->deviceid;

        switch(ev.xcookie.evtype)
        {
          case XI_Motion:
            //LOG(INFO) <<  "motion";
            setSignal(boost::lexical_cast<string>(deviceid) + "_x", evData->event_x);
            setSignal(boost::lexical_cast<string>(deviceid) + "_y", evData->event_y);
            VLOG(1) << deviceid << " " << evData->event_x << " " << evData->event_y;

            break;

          case XI_ButtonPress:
            VLOG(1) << deviceid << " button: " << evData->detail;
            setSignal(boost::lexical_cast<string>(deviceid) + "_" + 
                boost::lexical_cast<string>(evData->detail), 1);

            break;

          case XI_ButtonRelease:
            VLOG(1) << deviceid << " unclick";
            setSignal(boost::lexical_cast<string>(deviceid) + "_" + 
                boost::lexical_cast<string>(evData->detail), 0);
            break;
#if 0
          case XI_KeyPress:
            LOG(INFO) << "key down";
            break;

          case XI_KeyRelease:
            printf("key up\n");
            break;
#endif
        } // switch 
      } // correct event
      XFreeEventData(display, &ev.xcookie);
    
      return true;
    }


}

