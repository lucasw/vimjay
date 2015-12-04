/*

  Copyright 2013 Lucas Walter

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

#include "video.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <ros/console.h>

#include <deque>

#include "config.h"

using namespace cv;
using namespace std;

namespace bm
{
//////////////////////////////////////////////////

// TODO get rid of these and replace with generic ros tools for webcam and
// video playback (is there a video playback ros node, ffmpeg/mplayer to ros?)
// May want to just move these into standalone ros nodelets if the opencv
// webcam is better/more flexible for this than uvc cam or whatever else).
VideoCapture::VideoCapture(const std::string name) :
  ImageNode(name),
  error_count(0)
{

  ROS_INFO_STREAM(name << " video capture");
}

void VideoCapture::init()
{
  ImageNode::init();
}

///////////////////////////////////////////////

Video::Video(const std::string name) :
  Buffer(name)
  // VideoCapture(name)
{
}

void Video::init()
{
  Buffer::init();
  is_thread_dirty = false;
  setSignal("mode", 0, false, ROLL, 0, 4);
  // setString("file", "../data/test.mp4");
  setString("file", "../data/test.webm");

  ROS_INFO_STREAM(name << " video");
  cam_thread = boost::thread(&Video::runThread, this);
}

Video::~Video()
{
  run_thread = false;
  cam_thread.join();
}

void Video::runThread()
{
  run_thread = true;

  while (run_thread)
  {
    {
      bool is_valid, is_dirty;
      std::string file = getString("file", is_valid, is_dirty, 1);
      if (is_valid && is_dirty)
      {
        ROS_INFO_STREAM(name << " opening new video source "
                        << CLTXT << file << CLNRM);
        video.open(file);
        // TBD temp kinect test
        // video.open(cv::CAP_OPENNI);

        boost::mutex::scoped_lock l(frames_mutex);
        frames.clear();
      }
    }

    spinOnce();

    usleep(2000);

  } // while true
} // runThread

bool Video::spinOnce()
{
  // if (!VideoCapture::spinOnce()) return false;
  if (!video.isOpened())
  {
    usleep(10000);
    return false;
  }

  if (!getBool("enable"))
    return true;

  cv::Mat dst;
  if (!getVideoFrame(video, dst, name, getModeType(), getSignal("keep_aspect")))
    return false;

  add(dst, false);

  // TBD is this still necessary
  is_thread_dirty = true;

  if (false)
  {
    bool is_valid, is_dirty;
    float set_avi_ratio = getSignal("set_avi_ratio", is_valid, is_dirty, 1);
    if (is_valid && is_dirty)
    {
      ROS_INFO_STREAM(set_avi_ratio);
      video.set(CV_CAP_PROP_POS_AVI_RATIO, set_avi_ratio);
    }
  }

  setSignal("pos_msec",     video.get(CV_CAP_PROP_POS_MSEC));
  setSignal("pos_frames",   video.get(CV_CAP_PROP_POS_FRAMES));
  setSignal("avi_ratio",    video.get(CV_CAP_PROP_POS_AVI_RATIO));
  setSignal("frame_width",  video.get(CV_CAP_PROP_FRAME_WIDTH));
  setSignal("frame_height", video.get(CV_CAP_PROP_FRAME_HEIGHT));
  setSignal("fps",          video.get(CV_CAP_PROP_FPS));
  setSignal("frame_count",  video.get(CV_CAP_PROP_FRAME_COUNT));



  return true;
}

bool Video::update()
{
  // don't do Buffer update
  const bool rv = Node::update();
  if (!rv) return false;

  // set the out to whatever the ind input is asking for
  setOut();

  return true;
}


////////////////////////////////////////////////////////////
Webcam::Webcam(const std::string name) :
  VideoCapture(name)
{
}

void Webcam::init()
{
  VideoCapture::init();
  is_thread_dirty = false;
  setSignal("mode", 0, false, ROLL, 0, 4);

  cam_thread = boost::thread(&Webcam::runThread, this);
}

Webcam::~Webcam()
{
  ROS_INFO_STREAM(name << " stopping camera capture thread");
  run_thread = false;
  cam_thread.join();

  ROS_INFO_STREAM(name << " releasing the camera");
  video.release();
}

bool VideoCapture::spinOnce()
{

  if (!video.isOpened())
  {
    usleep(10000);
    return false;
  }

  if (!getBool("enable"))
    return true;

  // TBD check enable before grabbing
  // TBD the behavior for webcams is to grab continuously,
  // but maybe avi files should be buffered as quickly as
  // possible?  Doesn't work well for large videos?
  // Currently the video plays at very high speed in this
  // separate thread and there is no way to change the frame
  // Why duplicate the image buffer interface?  There may
  // be a good reason.

  cv::Mat dst;
  if (!getVideoFrame(video, dst, name, getModeType(), getSignal("keep_aspect")))
    return false;

  // out_lock.lock();
  setImage("out", dst);
  // TBD is this still necessary
  // is_thread_dirty = true;
  // out_lock.unlock();
  // } else {
  //  VLOG(3) << name << " dissimilar capture";
  //  out = new_out;
  // }

  // TBD out is the same address every time, why doesn't clone produce a new one?
  // VLOG(3) <<

  return true;
} // spinOnce

void Webcam::runThread()
{
  run_thread = true;
  // cv::namedWindow("webcam", CV_GUI_NORMAL);

  video.open(0);

  while (run_thread)
  {
    spinOnce();
  } // while run_thread

} // runThread

bool Webcam::update()
{
  // don't call ImageNode update because it will clobber the "out" image set in the thread
  if (!Node::update()) return false;
  // ImageNode::update();


  if (is_thread_dirty) setDirty();
  is_thread_dirty = false;

  return true;
}
}  // namespace bm

