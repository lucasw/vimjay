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
  int error_count;
  
  public:
  VideoCapture(const std::string name);
  virtual void init();
  //virtual ~VideoCapture();
  //
  
};

class Webcam : public VideoCapture
{
  bool run_thread;
  bool is_thread_dirty;

  void runThread();

  public:
  Webcam(const std::string name);
  virtual void init();
  virtual ~Webcam();

  virtual bool update();

};

/// multiple inheritance would maybe make this better
/// but doesn't seem like a good fit/ or requires upstream
/// changes I don't want to mess with right now.
class Video : public Buffer //public VideoCapture 
{
  cv::VideoCapture video; 
  boost::thread cam_thread;
  bool run_thread;
  bool is_thread_dirty;
  void runThread();
protected:
  virtual bool spinOnce();

public:
  Video(const std::string name);
  virtual ~Video();
  virtual void init();
  virtual bool update();
};



} // bm
#endif // MISC_NODES
