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

#include <glog/logging.h>

#include <deque>

#include "config.h"

using namespace cv;
using namespace std;

namespace bm {
//////////////////////////////////////////////////

  VideoCapture::VideoCapture(const std::string name) : 
      ImageNode(name),
      error_count(0)
  {

    LOG(INFO) << name << " video capture";
  }

  ///////////////////////////////////////////////

  Video::Video(const std::string name) : 
      VideoCapture(name)
  {
    is_thread_dirty = false;
    setSignal("mode", 0, false, ROLL, 0, 4);
    setString("file", "../data/test.mp4");
    
    LOG(INFO) << name << " video";
    cam_thread = boost::thread(&Video::runThread, this);
  }

  void Video::runThread()
  {

    while (true) {
      {
        bool is_valid, is_dirty;
        std::string file = getString("file", is_valid, is_dirty, 1);
        if (is_valid && is_dirty) {
          LOG(INFO) << name << " opening new video source " 
            << CLTXT << file << CLNRM;
          video.open(file);
        }
      }

    } // while true
  } // runThread

  bool Video::spinOnce() 
  {
    if (!VideoCapture::spinOnce()) return false;

      {
        bool is_valid, is_dirty;
        float set_avi_ratio = getSignal("set_avi_ratio", is_valid, is_dirty, 1);
        if (is_valid && is_dirty) {
          LOG(INFO) << set_avi_ratio;
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
    const bool rv = Node::update();
    if (!rv) return false;
   
    spinOnce();

    return true;
  }


  ////////////////////////////////////////////////////////////
  Webcam::Webcam(const std::string name) : 
      VideoCapture(name)
  {
    is_thread_dirty = false;
    setSignal("mode", 0, false, ROLL, 0, 4);

    cam_thread = boost::thread(&Webcam::runThread, this);
  }
  
  Webcam::~Webcam()
  {
    LOG(INFO) << name << " stopping camera capture thread";
    do_capture = false;
    run_thread = false;
    cam_thread.join();

    LOG(INFO) << name << " releasing the camera";
    video.release();
  }

bool getVideoFrame(
    cv::VideoCapture& video, 
    const std::string name, 
    const int mode_type,
    cv::Mat& dst) 
{
        if( !video.grab() )
        {
          // TBD this is only an error with a live webcam
          //LOG(ERROR) << name << " Can not grab images." << endl;
          //error_count++;
          return false;
        } 

        cv::Mat new_out;
        video.retrieve(new_out);

        if (new_out.empty()) {
          LOG(ERROR) << name << " new image empty";
          //error_count++;
          return false;
        }

        //error_count--;
        //if (error_count < 0) error_count = 0;
        // I think opencv is reusing a mat within video so have to clone it

        //if (&new_out.data == &out.data) {
        //
        //cv::Mat tmp; // new_out.clone();
       
        float scale = 1.0;
        if (MAT_FORMAT == CV_16S) scale = 255;
        if (MAT_FORMAT == CV_32F) scale = 1.0/255.0;
        if (MAT_FORMAT == CV_8U) scale = 1.0;
        cv::Mat tmp0 = cv::Mat(new_out.size(), CV_8UC4, cv::Scalar(0)); 
        // just calling reshape(4) doesn't do the channel reassignment like this does
        int ch[] = {0,0, 1,1, 2,2}; 
        mixChannels(&new_out, 1, &tmp0, 1, ch, 3 );
        cv::Mat tmp;
        tmp0.convertTo(tmp, MAT_FORMAT,scale); //, 1.0/(255.0));//*255.0*255.0*255.0));

        // TBD add black borders to preserve aspect ratio of original
        cv::Size sz = Config::inst()->getImSize();
        cv::resize(tmp, dst, sz, 0, 0, mode_type );
       
    return true;
  }

  bool VideoCapture::spinOnce()
  {

    if (!video.isOpened() ) {
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
    if (!getVideoFrame(video, name, getModeType(), dst)) 
      return false;

    //out_lock.lock();
    setImage("out", dst);
    // TBD is this still necessary
    is_thread_dirty = true;
    //out_lock.unlock();
    //} else {
    //  VLOG(3) << name << " dissimilar capture";
    //  out = new_out;
    //}

    // TBD out is the same address every time, why doesn't clone produce a new one?
    //VLOG(3) << 

    return true;
  } // spinOnce

  void Webcam::runThread()
  {
    do_capture = true;
    run_thread = true;
    //cv::namedWindow("webcam", CV_GUI_NORMAL);
   
    video.open(0);

    while (run_thread) {
      spinOnce();
    } // while run_thread

  } // runThread

  bool Webcam::update()
  {
    // don't call ImageNode update because it will clobber the "out" image set in the thread
    Node::update();
    //ImageNode::update();

    if ( is_thread_dirty ) setDirty();
    is_thread_dirty = false;

    return true;
  }


} //bm

