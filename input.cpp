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
#include "utility.h"

//using namespace cv;
using namespace std;




//DEFINE_string(mouse, "/dev/input/mouse0", "/dev/input/mouseN or /dev/input/eventN");
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

  // from git://gitorious.org/vinput/vinput.git demo-paint.c (GPL)
	/* Connect to the X server */
	Display *display = XOpenDisplay(NULL);


	/* Check if the XInput Extension is available */
	int opcode, event, error;
	if (!XQueryExtension(display, "XInputExtension", &opcode, &event, &error)) {
		printf("X Input extension not available.\n");
		return -1;
	}
  LOG(INFO) <<"XInputExtension available";

	/* Check for XI2 support */
	int major = 2, minor = 0;
	if (XIQueryVersion(display, &major, &minor) == BadRequest) {
		printf("XI2 not available. Server supports %d.%d\n", major, minor);
		return -1;
	}
  LOG(INFO) <<"XI2 available";

	/* Set up MPX events */
	XIEventMask eventmask;
	unsigned char mask[1] = { 0 };

	eventmask.deviceid = XIAllMasterDevices;
	eventmask.mask_len = sizeof(mask);
	eventmask.mask = mask;

	/* Events we want to listen for */
	XISetMask(mask, XI_Motion);
	XISetMask(mask, XI_ButtonPress);
	XISetMask(mask, XI_ButtonRelease);
	//XISetMask(mask, XI_KeyPress);
	//XISetMask(mask, XI_KeyRelease);

	/* Register events on the window */
  int screen_num = DefaultScreen(display);
  Window win = XCreateSimpleWindow(display, DefaultRootWindow(display),
          0, 0, 500, 500, 5, 
          WhitePixel(display, screen_num), 
          BlackPixel(display, screen_num));
  XMapWindow(display, win);
  XSync( display, False );

  XISelectEvents(display, win, &eventmask, 1);

  //
  cv::Mat tmp;
  // BGR
  tmp = cv::Mat(500,500, CV_8UC3, cv::Scalar(255,100,50));
  //cv::imshow("temp", tmp);
  //XImage* ximage = XGetImage(display, win, 0, 0, 500, 500, AllPlanes, ZPixmap);
  XImage* ximage = XGetImage(display, DefaultRootWindow(display), 0, 0, 500, 500, AllPlanes, ZPixmap);
  Screen* screen = DefaultScreenOfDisplay(display);
  //XImage* ximage = XCreateImage(display, DefaultVisual(display, screen) 
  bm::matToXImage(tmp, ximage, win, *display, *screen);
  GC gc = XCreateGC(display, win, 0, NULL);
  XPutImage(display, win,  gc, ximage, 0, 0, 0, 0, 500, 500);
  
  cv::waitKey(10);

	/* Event loop */
	while(1) {

		XEvent ev;
		/* Get next event; blocks until an event occurs */
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
				LOG(INFO) <<  "motion";
        LOG(INFO) << deviceid << " " << evData->event_x << " " << evData->event_y;
				
				break;

			case XI_ButtonPress:
		    LOG(INFO) << deviceid << " button: " << evData->detail;
			//	printf("abs X:%f Y:%f - ", evData->root_x, evData->root_y);
			//	printf("win X:%f Y:%f\n", evData->event_x, evData->event_y);
		
				break;

			case XI_ButtonRelease:
				LOG(INFO) << deviceid << " unclick";
				break;
  #if 0
			case XI_KeyPress:
				LOG(INFO) << "key down";
				break;

			case XI_KeyRelease:
				printf("key up\n");
				break;
        #endif
			}
		}
		XFreeEventData(display, &ev.xcookie);
	}

	return 0;


  return 0;
}

