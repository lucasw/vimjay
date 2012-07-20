
#include <iostream>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>


#include <deque>
//#include <pair>

using namespace cv;
using namespace std;

namespace bm {

// informal timer for the system
int counter;

class Node
{
  public:
  // has this node been updated this timestep, or does it need to be updated this timestep
  // because of dependencies
  bool do_update;
  // is the output of this node different from the last  timestep
  bool is_dirty;

  Node() {
    do_update = false;
    is_dirty = true;
  }
  
  // TBD vector of input nodes
  vector<Node*> inputs;

  virtual void update() 
  {
    if (!do_update) return;

    do_update = false; 

    bool inputs_dirty = false;
    for (int i = 0; i < inputs.size(); i++) {
      inputs[i].update();
      if (inputs[i].is_dirty) inputs_dirty = true;
    }

    // there inheriting object needs to set is_dirty as appropriate?
    is_dirty = inputs_dirty;
  }
};

class ImageNode : public Node
{
  cv::Mat out;

public:
    
  ImageNode() : Node()
  {

  }

  // TBD could there be a templated get function to be used in different node types?
  virtual cv::Mat get() {
    return out;
  }
}

// TBD subclasses of Node that are input/output specific, or make that general somehow?

class Signal : public Node
{
  public:
  Signal(const float new_step=0.01, const float offset=0.0) : Node()
  {
    value = offset;
    step = new_step;
    LOG(INFO) << "Signal " << value << " " << new_step;
  }
  
  virtual void update() 
  {
    Node::update();

    value += step;
    if (value > 1.0) value = 0.0;
    if (value < 0.0) value = 1.0;
    is_dirty = true;
  }
  
  float value;
  float step;
};

class Saw : public Signal
{
  public:
  Saw(const float new_step=0.01, const float offset=0.0) : Signal(new_step, offset) {}

  virtual void update()
  { 
    Node::update();

    value += step;
    if (value > 1.0) {
      step = -abs(step);
      value = 1.0;
    }
    if (value < 0.0) {
      step = abs(step);
      value = 0.0;
    }
    is_dirty = true;
    //LOG(INFO) << step << " " << value;
  }
};

///
class Buffer : public ImageNode
{

  deque<cv::Mat> frames;
  int max_size;
  
  public:

  Buffer(const int max_size=180) : Node() {
    this->max_size = max_size;
    LOG(INFO) << "new buffer max_size " << this->max_size;

  }
  

  void add(cv::Mat new_frame)
  {
    frames.push_back(new_frame);

    while (frames.size() >= max_size) frames.pop_front();

    // TBD is_dirty wouldn't be true for callers that want frames indexed from beginning if no pop_front has been done.
    is_dirty = true;
  }

  cv::Mat get(const float fr) 
  {
    if (frames.size() < 1) {
      VLOG(1) << "no frames returning gray";
      cv::Mat tmp = cv::Mat(640, 480, CV_8UC3);
      tmp = cv::Scalar(128);
      return tmp;
    }
    int ind = (int)(fr*(float)frames.size());
    if (fr < 0) {
      ind = frames.size() - ind;
    }
    
    if (ind > frames.size() -1) ind = frames.size()-1;
    if (ind < 0) ind = 0;
    //ind %= frames.size();
   
    //VLOG_EVERY_N(1,10) 
    //LOG_EVERY_N(INFO, 10) << ind << " " << frames.size();
    return frames[ind];
  }


};

///////////////////////////////////////////////////////////
class Tap : public ImageNode
{
  public:

  Signal* signal;
  Buffer* buffer;

  bool changed;
  cv::Mat out;

  Tap(Signal* new_signal =NULL, Buffer* new_buffer=NULL) : Node()
  {
    signal = new_signal;
    buffer = new_buffer;

    inputs.push_back(signal);
    inputs.push_back(buffer);
  }

  virtual void update()
  {
    Node::update();

    if (is_dirty) {
      out = buffer->get(signal->value);
    }
  }

  // this is sort of strange, maybe should have another object that can be many to one with the buffer?
  /*
   * virtual cv::Mat get()
  {
    //if (rv.empty())
    if (VLOG_IS_ON(2)) {
    cv::line(rv, cv::Point(0,0), cv::Point( rv.cols, 0), cv::Scalar(0,0,0), 2);
    cv::line(rv, cv::Point(0,0), cv::Point( signal->value* rv.cols, 0), cv::Scalar(255,0,0), 2);
    }

    return out;
  }*/
};

class Add : public Tap
{
  public:
  
  // TBD make a vector?
  Tap* p1;
  float f1;

  Tap* p2;
  float f2;
  
  cv::Mat

  Add(Tap* np1, Tap* np2, float nf1 = 0.5, float nf2 = 0.5) : Node()
  {
    p1 = np1;
    p2 = np2;

    f1 = nf1;
    f2 = nf2;
   
    inputs.push_back(p1);
    inputs.push_back(p2);
  }

  virtual void update()
  {
    Node::update();

    if (is_dirty) {
      out = p1->get() * f1 + p2->get() * f2;
    }
  }

};


  ////////////////////////////////////
  class CamThing
  {
    // make sure all Nodes are stored here
    deque<Node*> all_nodes;

    // conveniently create and store node
    template <class nodeType>
    nodeType* getNode()
    {
        nodeType* node = new nodeType();
        all_nodes.push_back(node);
        return node;
    }

    void clearAllNodeUpdates() 
    {
      for (int i = 0; i < all_nodes.size(); i++) {
        all_nodes[i]->do_update = false;
      }
    }

    CamThing() 
    {
  LOG(INFO) << "camera opening ...";
  VideoCapture capture(0); //CV_CAP_OPENNI );
  LOG(INFO) << "done.";

  int count = 0;

  if( !capture.isOpened() )
  {
    LOG(ERROR) << "Can not open a capture object.";
    return;// -1;
  }
  bool rv1 = capture.set( CV_CAP_PROP_FRAME_WIDTH, 800);
  bool rv2 = capture.set( CV_CAP_PROP_FRAME_HEIGHT, 600);
  LOG(INFO) << "set res " << rv1 << " " << rv2;

  
  const float advance = 0.2;
  
  Buffer* cam_buf = new Buffer(1.0/advance*5);
  
  Signal* s1 = new Saw(advance);
  Tap* p1 = new Tap(s1, cam_buf);
   
  // make a chain, sort of a filter
  for (float ifr = advance; ifr <= 1.0; ifr += advance ) {

    Signal* s2 = new Saw(advance, ifr);
    Tap* p2 = new Tap(s2, cam_buf);

    Add* add = new Add(p1, p2, 0.5, 0.5);
    
    /*
    Signal* s3 = new Saw(advance, ifr -advance*2.5);
    Tap* p3 = new Tap(s2, cam_buf);

    add = new Add(add, p3, 2.0, -1.0);
    */
    p1 = add;
  }
  

  cv::namedWindow("cam", CV_GUI_NORMAL);
  cv::moveWindow("cam",0,0);
  cv::namedWindow("out", CV_GUI_NORMAL);
  cv::moveWindow("out",640,0);
 
  // get the first black frames out
  capture.grab();
  capture.grab();
  }

  void update() {

    if( !capture.grab() )
    {
      cout << "Can not grab images." << endl;
      continue;
      //return -1;
    }
    
    {
      cv::Mat frame;
      capture.retrieve(frame); 
      if (frame.empty()) {
        cout << "bad capture" << endl;
        continue;
      }
      // I think opencv is reusing a mat within capture so have to clone it
      cam_buf->add(frame.clone());

      // TBD put this in different thread 
      {
        p1->update();
      }
    }

    if( waitKey( 10 ) == 'q' )
      break;
  }


  
  void draw() {
 
    imshow("cam",frame);
    imshow("out", p1->get());
  }

  };

};
/*
 * To work with Kinect the user must install OpenNI library and PrimeSensorModule for OpenNI and
 * configure OpenCV with WITH_OPENNI flag is ON (using CMake).

TBD have a mode that takes a webcam, uses brightness as depth, and thresholds it for the valid map

 */
int main( int argc, char* argv[] )
{
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  // pair of rgb images and depths put together
  
  bm::CamThing* cam_thing = new bm::CamThing();
  
  for(;;) {
    cam_thing->update();
    cam_thing->draw();
  }

  return 0;
}

