#ifndef __VIDEO_H__
#define __VIDEO_H__

#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//#include <random>
#include <deque>
#include <map>

#include "nodes.h"

namespace bm {

class VideoCapture : public ImageNode
{

  protected:
  cv::VideoCapture video; 
  boost::thread cam_thread;
  virtual bool spinOnce();
  bool is_thread_dirty;
  int error_count;
  
  public:
  VideoCapture(const std::string name);
  //virtual ~VideoCapture();
  //
  
};

class Webcam : public VideoCapture
{
  bool do_capture;
  bool run_thread;

  void runThread();

  public:
  Webcam(const std::string name);
  virtual ~Webcam();

  virtual bool update();

};

class Video : public VideoCapture 
{
  void runThread();
protected:
  virtual bool spinOnce();

public:
  Video(const std::string name);
  //virtual ~Video();
  virtual bool update();
};



} // bm
#endif // MISC_NODES
