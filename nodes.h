#ifndef __NODES_H__
#define __NODES_H__

#include <iostream>
#include <stdio.h>

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

// informal timer for the system

class Node
{
  // this structure tracks arbitrary numbers of callers to see if there have been
  // change since the last call
  std::map<void*, std::map<int, bool> > dirty_hash;  
  public:
  // has this node been updated this timestep, or does it need to be updated this timestep
  // because of dependencies
  bool do_update;
  
  bool enabled;

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
    
  bool setUpdate();
  
  // the rv is so that an ineritanning function will know whether to 
  // process or not
  virtual bool update(); 

  virtual bool draw(float scale = 0.125); 

};

class ImageNode : public Node
{
public:
  cv::Mat out;
  cv::Mat out_old;
    
  ImageNode();// : Node()

  virtual bool update();
  // TBD could there be a templated get function to be used in different node types?
  virtual cv::Mat get();


  virtual bool draw(float scale = 0.2);
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

};

class Webcam : public ImageNode
{

  cv::VideoCapture capture; //CV_CAP_OPENNI );
  void runThread();
  bool is_thread_dirty;
  bool do_capture;

  public:
  Webcam();

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

  float min;
  float max;
  float value;
  float step;
};

bool getValue(std::vector<Node*>& inputs, const int ind, float& val);

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

};

///////////////////////////////////////////////////////////
class Tap : public ImageNode
{
  public:

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
  
  // TBD make a vector?
  ImageNode* p1;
  float f1;

  ImageNode* p2;
  float f2;
  
  Add(); // : ImageNode()
  
  void setup(ImageNode* np1, ImageNode* np2, float nf1 = 0.5, float nf2 = 0.5);

  virtual bool update();
};


#if 0
////////////////////////////////////
class CamThing
{
  // make sure all Nodes are stored here
  deque<Node*> all_nodes;

  // the final output 
  // TBD make this a special node type
  ImageNode* output;

  public:

  // conveniently create and store node
  template <class nodeType>
    nodeType* getNode(string name = "", cv::Point loc=cv::Point(0.0, 0.0));
 
  void clearAllNodeUpdates(); 

  VideoCapture capture; //CV_CAP_OPENNI );
  Buffer* cam_buf;  
  int count;

  cv::Mat test_im;

  CamThing();
  
  cv::Mat cam_image;
  cv::Mat graph;
  bool do_capture;

  bool update();
  
  void draw();
  
  };


#endif

};
#endif // ifdef __NODES_H__
