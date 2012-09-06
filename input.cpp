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
#include <time.h>
#include <typeinfo>
#include <cxxabi.h> // non portable

#include <deque>
//#include <pair>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "nodes.h" 
#include "misc_nodes.h" 
#include "signals.h"
#include "filter.h"
#include "generate.h"
#include "screencap.h"

using namespace cv;
using namespace std;

#include <linux/input.h>
#include <fcntl.h>

DEFINE_string(mouse, "/dev/input/mouse0", "/dev/input/mouseN or /dev/input/eventN");
/*
 * To work with Kinect the user must install OpenNI library and PrimeSensorModule for OpenNI and
 * configure OpenCV with WITH_OPENNI flag is ON (using CMake).

TBD have a mode that takes a webcam, uses brightness as depth, and thresholds it for the valid map

 */
int main( int argc, char* argv[] )
{
 // google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  google::ParseCommandLineFlags(&argc, &argv, false);
 
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

  return 0;
}

