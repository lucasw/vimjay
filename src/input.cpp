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

#include "vimjay/input.h"

#include <boost/lexical_cast.hpp>
#include <cxxabi.h>  // non portable
#include <deque>
#include <fcntl.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/console.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <time.h>
#include <typeinfo>
#include <utility>
#include <vector>
// #include <pair>

#include "vimjay/config.h"
#include "vimjay/filter.h"
#include "vimjay/generate.h"
#include "vimjay/misc_nodes.h"
#include "vimjay/nodes.h"
#include "vimjay/screencap.h"
#include "vimjay/signals.h"
#include "vimjay/utility.h"

namespace bm
{
// DEFINE_int32(width, 640, "");
// DEFINE_int32(height, 480, "");
// DEFINE_string(mouse, "/dev/input/mouse0", "/dev/input/mouseN or /dev/input/eventN");
/*
 * To work with Kinect the user must install OpenNI library and PrimeSensorModule for OpenNI and
 * configure OpenCV with WITH_OPENNI flag is ON (using CMake).

TBD have a mode that takes a webcam, uses brightness as depth, and thresholds it for the valid map

 */
#if 0
  int fd;
  // cat /proc/bus/input/devices - TBD how to find just the mouses?
  // - TBD how to find just the mouses?
  // TBD can't get touchpad to work, don't even see it when catting /dev/input/mouseN
  if ((fd = open(FLAGS_mouse.c_str(), O_RDONLY)) < 0)
  {
    ROS_ERROR_STREAM("couldn't open mouse " << fd);
    exit(0);
  }
  struct input_event ev;

  bool rv = true;
  while (rv)
  {
    read(fd, &ev, sizeof(struct input_event));
    ROS_DEBUG_STREAM_COND(log_level > 1, "value 0x" << std::hex << ev.value
                          << ", type 0x" << std::hex << ev.type
                          << ", code 0x" << std::hex << ev.code);
    if (ev.type == EV_REL)
    {
      if (ev.value != 0)
      {
        // mouse move left
        if (ev.code == ABS_X) ROS_INFO_STREAM("dx " << ev.value);
        // mouse move right
        if (ev.code == ABS_Y)  ROS_INFO_STREAM("dy " << ev.value);
        // wheel
        if (ev.code == REL_WHEEL) ROS_INFO_STREAM("wheel " << ev.value);
      }
    }
    if (ev.type == EV_MSC)
    {
      // 0x90001 - 3
      ROS_INFO_STREAM("Button value 0x" << std::hex << ev.value
                      << ", code " << ev.code);
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
  // bm::setWindowDecorations(display, win,false);

  cv::Mat tmp;
  // BGR
  tmp = cv::Mat(height, width, CV_8UC4, cv::Scalar(255, 100, 50));

  bm::matToScreen(tmp, display, win);

  // cv::imshow("temp", tmp);
  // cv::waitKey(10);

  /* Event loop */
  while (1)
  {
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

    // event_thread = boost::thread(&Mouse::runThread, this);
  }

  Mouse::~Mouse()
  {
    // run_thread = false;
    // event_thread.join();
  }
#if 0
  bool Mouse::update()
  {
    if (!Node::update()) return false;

    return true;
  }
#endif

// mouse input takes place in draw thread instead of update
// because the draw thread 'owns' the display/window
  bool Mouse::draw(cv::Point2f ui_offset)
  {
    Node::draw(ui_offset);
    // run_thread = true;

    vector<string> sig_name;
    vector<float> sig_val;
    std::vector< std::pair<char, bool> > keys;

    /// TBD it would be nice to be able to get the display
    /// of the root window so all mouse moves can be in here
    /// or optionally the display of the preview window, or
    /// gui window
    if (!getMouse(display, opcode, sig_name, sig_val, keys))
      return false;

    for (size_t i = 0; i < sig_name.size(); i++)
    {
      setSignal(sig_name[i], sig_val[i]);
    }

    for (size_t i = 0; i < keys.size(); i++)
    {
      ROS_INFO_STREAM(keys[i].first);
      stringstream ss;
      ss << keys[i].first;
      setSignal("key_" + ss.str(), keys[i].second);
    }

    return true;
  }

#if 0
  MouseSignal::MouseSignal(const std::string name) :
    SigBuffer(name),
    display(NULL)
  {
    sigs.resize(Config::inst()->im_width);
  }

  bool MouseSignal::draw()
  {
    vector<string> sig_name;
    vector<float> sig_val;
    // TBD getMouse is completely unsuitable for this
    if (!getMouse(display, opcode, sig_name, sig_val))
      return false;



    return true;
  }
#endif

  GamePad::GamePad(const std::string name) :
    ImageNode(name)
  {
  }

  void GamePad::init()
  {
    ImageNode::init();

    is_initted = false;
    // setSignal("0_x", 0);

    // event_thread = boost::thread(&Mouse::runThread, this);

    // TBD allow changing of joystick input- could allow numeric input
    // to append to js,

    unsigned char axes = 2;
    unsigned char buttons = 2;
    int version = 0x000800;
    const int NAME_LENGTH = 128;
    char name[NAME_LENGTH] = "Unknown";

    if ((fd = open("/dev/input/js0", O_RDONLY)) < 0)
    {
      perror("controlrecorder");
      setString("name", "NO GAMEPAD");
      return;
    }

    ioctl(fd, JSIOCGVERSION, &version);
    ioctl(fd, JSIOCGAXES, &axes);
    ioctl(fd, JSIOCGBUTTONS, &buttons);
    ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);

    fcntl(fd, F_SETFL, O_NONBLOCK);

    axis.resize(axes);
    button.resize(buttons);


    const bool internally_set = true;
    setString("name", name);
    setSignal("version_0", (version >> 16) & 0xff, internally_set);
    setSignal("version_1", (version >> 8)  & 0xff, internally_set);
    setSignal("version_2", version & 0xff, internally_set);
    setSignal("axes", axes, internally_set);
    setSignal("buttons", buttons, internally_set);

    for (size_t i = 0; i < axis.size(); i++)
    {
      setSignal("axis_" + boost::lexical_cast<string>(i), axis[i],
                internally_set);
    }
    for (size_t i = 0; i < button.size(); i++)
    {
      setSignal("button_" + boost::lexical_cast<string>(i), button[i],
                internally_set);
    }

    is_initted = true;

    joy_thread = boost::thread(&GamePad::runThread, this);
  }

  GamePad::~GamePad()
  {
    // run_thread = false;
    // event_thread.join();
  }

  void GamePad::runThread()
  {
    ROS_INFO_STREAM("Starting gamepad thread");

    run_thread = true;

    while (run_thread)
    {
      // ROS_INFO_ONCE("game pad run loop " << fd);
      if (fd > 0)
      {
        // ROS_INFO_ONCE("game pad fd " << fd);
        // TBD put int thread
        while (
          read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event))
        {
          const int js_num = static_cast<int>(js.number);
          // ROS_INFO_ONCE("game pad fd");

          switch (js.type & ~JS_EVENT_INIT)
          {
          case JS_EVENT_BUTTON:
          {
            if (js.number < button.size())
              button[js.number] = js.value;

            const std::string button_name =
              "button_" + boost::lexical_cast<string>(js_num);
            setSignal(button_name, js.value);
            ROS_DEBUG_STREAM_COND(log_level > 1, "button " << button_name << " " << js_num << " " << js.value);

            break;
          }

          case JS_EVENT_AXIS:
          {
            if (js.number < axis.size())
              axis[js.number] = js.value;

            const std::string axis_name =
              "axis_" + boost::lexical_cast<string>(js_num);
            // TBD scale automatically for now
            setSignal(axis_name, 10.0 * js.value / 32768.0);
            ROS_DEBUG_STREAM_COND(log_level > 1, "axis " << axis_name << " " << js_num << " " << js.value);
            break;
          }
          }
        }

        usleep(1000);
      }
      else
      {
        ROS_WARN_ONCE("no joystick connectect yet");
        sleep(1);
      }
    }  // run_thread
  }  // runThread

  bool GamePad::update()
  {
    if (!Node::update()) return false;

    return true;
  }

  bool GamePad::draw(cv::Point2f ui_offset)
  {
    return ImageNode::draw(ui_offset);
  }
}  // namespace bm

