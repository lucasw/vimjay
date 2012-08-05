#ifndef __NODES_H__
#define __NODES_H__

#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <deque>
#include <map>

#define MAT_FORMAT_C3 CV_8UC3
#define MAT_FORMAT CV_8U
//#define MAT_FORMAT_C3 CV_16SC3
//#define MAT_FORMAT CV_16S
//#define MAT_FORMAT_C3 CV_32FC3
//#define MAT_FORMAT CV_32F

namespace bm {

// TBD need threshold filter
// resize (or build resizing into input/output
// image directory loading- just a no input buffer after finished (need to resize every loaded image)

// more advanced:
// Buffer inventory- make it easy to put any buffer into the inventory (deepish copy)
// then make them easy to swap it into place where any other buffer exists
// the idea is to navigate to a buffer on input or output, save it to inventory, then swap it into a buffer sourcing
// some patch 

// nesting, keys for moving up or down
//
// yaml loading/saving
//
//kinect support
class Node
{
  // this structure tracks arbitrary numbers of callers to see if there have been
  // change since the last call
  std::map<void*, std::map<int, bool> > dirty_hash;  
  public:
  // has this node been updated this timestep, or does it need to be updated this timestep
  // because of dependencies
  bool do_update;
  
  bool enable;

  // is the output of this node different from the last  timestep
  //bool is_dirty;
  // has the node changed since the last time the pointer parameter supplied has called this function (and cleared it)
  bool isDirty(void* caller, int ind=0,  bool clear_dirty=true);
  
  bool setDirty();

  std::string name;
  cv::Point loc;
  cv::Mat graph;
  cv::Scalar vcol; 
  
  std::vector<Node*> inputs;
  
  Node();
  virtual ~Node() {}
    
  bool setUpdate();
  
  // the rv is so that an ineritanning function will know whether to 
  // process or not
  virtual bool update(); 

  virtual bool draw(float scale = 0.125); 

  virtual bool save(cv::FileStorage& fs);
  virtual bool load(cv::FileNodeIterator nd);
};

bool getValue(std::vector<Node*>& inputs, const int ind, float& val);

class ImageNode : public Node
{
public:
  cv::Mat out;
  cv::Mat out_old;
  cv::Mat tmp; // scratch image
    
  ImageNode();// : Node()

  virtual bool update();
  // TBD could there be a templated get function to be used in different node types?
  virtual cv::Mat get();


  virtual bool draw(float scale = 0.2);
};

// TBD allow multiple?
class Output : public ImageNode
{
  public:
  Output() {}

  // doesn't have anything special, just a class to be detected with a dynamic_cast upon loading
};

class Rot2D : public ImageNode
{
  public:
 
  // TBD these need to be Node inputs?
  float angle;
  float scale;
  cv::Point2f center;

  Rot2D();

  virtual bool update();

  //virtual bool save(cv::FileStorage& fs);
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


// TBD subclasses of Node that are input/output specific, or make that general somehow?

class Signal : public Node
{
  public:
  Signal(); // : Node()

  void setup(const float new_step=0.01, const float offset=0.0, const float min = 0.0, const float max=1.0); 
 
  virtual bool update();
  virtual bool draw(float scale);

  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);

  float min;
  float max;
  float value;
  float step;
};


class Saw : public Signal
{
  public:
  Saw(); // : Signal()
  
  void setup(const float new_step=0.01, const float offset=0.0, const float min =0.0, const float max=1.0); 
  
  virtual bool update();
  
};

////////////////////////////////
class Buffer : public ImageNode
{
  protected: 
  std::deque<cv::Mat> frames;
  
  public:

  Buffer(); 
  
  int max_size;
 
  virtual bool update();

  void add(cv::Mat new_frame);
  virtual bool draw(float scale); 
   
  virtual cv::Mat get();

  cv::Mat get(const float fr);

  // TBD get(int ind), negative ind index from last
  
  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);

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

  // TBD cast these from inputs[] instead?
  Signal* signal;
  Buffer* buffer;

  bool changed;

  Tap();// : ImageNode()

  void setup(Signal* new_signal =NULL, Buffer* new_buffer=NULL); 
  
  virtual bool update();
};

class Add : public ImageNode
{
  public:
  
  // TBD make a vector of ImageNodes so dynamic_casts don't need to be used?
  std::vector<float> nf;
  
  Add(); // : ImageNode()
  
  // TBD could require pair be passed in to enforce size
  void setup(std::vector<ImageNode*> np, std::vector<float> nf); 

  virtual bool update();
  
  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);
};

};
#endif // ifdef __NODES_H__
