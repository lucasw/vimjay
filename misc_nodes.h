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

class Rot2D : public ImageNode
{
  public:
  Rot2D();
  virtual bool update();
};

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
  Webcam();
  virtual ~Webcam();

  virtual bool update();

};


/////////////////////////////////
class ImageDir : public Buffer
{
  public:

  ImageDir() {}

  std::string dir;
  
  bool loadImages();

  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);
};

///////////////////////////////////////////////////////////
class Tap : public ImageNode
{
  public:

  bool changed;

  //float value;

  Tap();// : ImageNode()

  void setup(Signal* new_signal =NULL, Buffer* new_buffer=NULL); 
  
  virtual bool update();
  
  virtual bool draw();
};

class TapInd : public Tap
{
  public:

  TapInd() {}// : ImageNode()
  
  // TBD make an sval?
  //int ind;
  virtual bool update();
  virtual bool draw();
};

// Arbitrary inputs 
class Add : public ImageNode
{
  public:
  Add(); // : ImageNode()
  // TBD could require pair be passed in to enforce size
  // TBD get rid of this?
  void setup(std::vector<ImageNode*> np, std::vector<float> nf); 
  virtual bool update();
  virtual bool handleKey(int key);
};

class Multiply : public ImageNode
{
  public:
  Multiply(); 
  virtual bool update();
};

// Double inputs
class AbsDiff : public ImageNode
{
  public:
  AbsDiff(); 
  virtual bool update();
};

class Greater : public ImageNode
{
  public:
  Greater(); 
  virtual bool update();
};

// Single inputs
class Resize : public ImageNode
{
  public:
  Resize();
  virtual bool update();
};

class Flip : public ImageNode
{
  public:
  Flip();
  virtual bool update();
};



} // bm
#endif // MISC_NODES
