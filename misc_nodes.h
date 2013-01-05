#ifndef __MISC_NODES_H__
#define __MISC_NODES_H__

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
class Webcam : public ImageNode
{

  cv::VideoCapture capture; //CV_CAP_OPENNI );
  void runThread();
  bool is_thread_dirty;
  bool do_capture;
  bool run_thread;
  boost::thread cam_thread;

  int error_count;

  public:
  Webcam(const std::string name);
  virtual ~Webcam();

  virtual bool update();

};


/////////////////////////////////
class ImageDir : public Buffer
{
  std::deque<cv::Mat> frames_orig;
  vector<string> all_files;
  bool resizeImages();

  public:

  ImageDir(const std::string name);

  bool loadImages();

  virtual bool update();
  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);
};



} // bm
#endif // MISC_NODES
