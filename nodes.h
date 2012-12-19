#ifndef __NODES_H__
#define __NODES_H__

#include <iostream>
#include <sstream>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <deque>
#include <map>

#include "utility.h"

namespace bm {

static int ind;
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
//kinect support

// Basic nodes are ImageNodes, Signals (numerical), and image Buffers (TBD signal Buffers?)
// second types are vectors of image nodes and signals (probably required to be the same size,
// maybe they could be bundled into std::pairs)
//
// most nodes need to have dedicated input locations for those types so incoming connections know where to go
// each input also might want a string name?
// 
// it was nice to lump all types together in one inputs vector, but that doesn't scale right- should that
// be maintained (where disconnections are managed across both the vector of inputs and the dedicated input
// type connetions?)  Or use pointers to pointers?
//  what about having a map of maps for the inputs - inputs[SIGNAL]["min"]


static bool bool_val;

class Node;
//class Buffer;

// TBD need to template this
enum conType {  
  NONE,
  SIGNAL,
  IMAGE,
  BUFFER,
  SIGBUF,
  STRING
  // TBD STRBUF
};

enum satType {
  SAT_NONE = 0,
  SATURATE,
  ROLL
};

// base class for Connector and Node to inherit from
class Elem
{
  protected:

  // this structure tracks arbitrary numbers of callers to see if there have been
  // change since the last call
  std::map<const void*, std::map<int, bool> > dirty_hash;  
 
  public:
  //Elem();
  Elem(const std::string name);

  std::string name;
  std::string description;
  // is the output of this node different from the last  timestep
  //bool is_dirty;
  // has the node changed since the last time the pointer 
  // parameter supplied has called this function (and cleared it)
  bool isDirty(
      const void* caller, 
      const int ind=0, 
      const bool clear_dirty=true);
  
  bool setDirty();

  // utility bools to control visualization of the element
  bool highlight;
  bool highlight2;

  // has this node been updated this timestep, or does it need to be updated this timestep
  // because of dependencies
  bool do_update;
  virtual bool update();
};

// TBD original chose this type because of access convenience, but since 
// accessor function prevent others from using it directly then the convenience 
// is irrelavent.  Probably should just have a vector of structs
class Connector : public Elem
{
  protected:

  public:
  Connector(const std::string name);

  virtual bool update();

  // whether the dirtiness should force an update of the associated node
  // determined by usage, not explicitly set
  // set to true if this value will not be connected to externally, and will
  // be set internally
  bool internally_set;
  
  // the Connector that is sourcing this one, if any
  // this is somewhat odd, the connector is in three parts- the src, this, and dst
  // and each has a copy of the data
  Connector* src;
  Connector* dst;

  Node* parent;

  // src types
  conType type;
 
  cv::Point2f loc;
  
  std::vector<cv::Point2f> connector_points;

  bool setImage(cv::Mat im);

  cv::Mat getImage();

  void draw(cv::Mat, cv::Point2f ui_offset);

  // TBD could even have a float val or Mat here to store the last value
  // only used if conType == Signal, TBD subclass?
  float value;

  int saturate;
  float val_min;
  float val_max;

  // only used if conType == Image or Buffer
  cv::Mat im;

  std::string str;

  // TBD could store a copy of a sigbuf here
  // std::vector<float> sigbuf;

  boost::mutex im_mutex;

  //Buffer* getBuffer();
};

//typedef std::map<std::string, std::pair<Node*, std::string> > inputsItemType;
//typedef std::map<std::string, inputsItemType > inputsType;

class Node : public Elem
{
  protected:
  
  // velocity and acceleration of node screen position 
  bool posUpdate();
  boost::mutex port_mutex;

  public:
  
  cv::Point2f acc;
  cv::Point2f vel;

  std::vector<Connector*> ports;
  int getIndFromPointer(Connector* con);
  bool selectPortByInd(const int ind);
 

  cv::Point2f loc;
  // TBD get rid of this and pass it to draw every time
  cv::Mat graph_ui;
  cv::Scalar vcol;

  // these are for ui display purposes, shows the current potential connection
  int selected_port_ind;
  std::string selected_port;
  conType selected_type;
  bool draw_selected_port;
  //void drawSelectedPort();
  
  Node(const std::string name);

  //Node(std::string name, cv::Point loc, cv::Mat graph_ui ); 
  
  // TBD need to delete all the connectors
  virtual ~Node() {}
    
  bool setUpdate();
  
  // the rv is provided so that an inheriting function will know whether to 
  // process or not
  virtual bool update(); 

  virtual bool draw(cv::Point2f ui_offset); 

  virtual bool save(cv::FileStorage& fs);
  virtual bool load(cv::FileNodeIterator nd);

  bool getPrevPort(const conType type=NONE);
  bool getNextPort(const conType type=NONE); 
  
  bool getInputPort(
      const conType type, 
      const std::string port,
      Connector*& con,
      std::string& src_port);

  void setInputPort(
      const conType type, 
      const std::string port,
      Node* src_node = NULL,
      const std::string src_port = "" 
    );

  // TBD calling any of these will create the input, so outside
  // nodes probably shouldn't call them?
  cv::Mat getImage(
      const std::string port,
      bool& valid = bool_val,
      bool& is_dirty = bool_val);//,
      //bool& is_dirty);
      //const bool require_dirty= false);
  
  // set image, only succeeds if not an input TBD - rw permissions?
  bool setImage(const std::string port, cv::Mat& im, const bool internally_set=false);

  float getSignal(
      const std::string port, 
      bool& valid = bool_val);

  bool setSignal(const std::string port, 
      const float val=0.0,
      const bool internally_set = false,
      const int saturate= SAT_NONE,
      const float min=0.0,
      const float max=1.0);

  // TBD rename these, they are really getBufferImage or similar
  cv::Mat getBuffer(
    const std::string port,
    const float val,
    int& actual_ind = ind
    );
    //cv::Mat& image);

  cv::Mat getBuffer(
    const std::string port,
    const int val,
    int& actual_ind = ind
    );
    //cv::Mat& image);

  //cv::Mat getBuffer(
  //  const std::string port,
  
  // getSigBuf()

  // There is no way to store a Buffer outside of a node, so all setting does is creating a connector port
  bool setBuffer(const std::string port, const bool internally_set=false);
  
  bool setSigBuf(const std::string port, const bool internally_set=false);

  bool setString(const std::string port, const std::string new_str);
  std::string getString(const std::string port,
      bool& valid = bool_val);

  virtual bool handleKey(int key);
};

/////////////////////////////////////////////////
// TBD split the following into new file
//////////////////////////////////////////////////

class ImageNode : public Node
{
public:
  // TBD make all three private
  //cv::Mat out;
  //cv::Mat out_old;
  //cv::Mat tmp; // scratch image
  //int write_count;
    
  ImageNode(const std::string name);// : Node()
  //ImageNode(std::string name, cv::Point loc, cv::Mat graph_ui ); 

  virtual bool update();

  virtual bool draw(cv::Point2f ui_offset);
  
  virtual bool handleKey(int key);
  
  std::stringstream dir_name;
  virtual bool writeImage();

  int getModeType(); // TBD supply string optionally
  int getBorderType(const bool avoid_wrap = false);
};

// TBD subclasses of Node that are input/output specific, or make that general somehow?

class Signal : public Node
{
  public:
  Signal(const std::string name); // : Node()

  void setup(const float new_step=0.01, const float offset=0.0, const float min = 0.0, const float max=1.0); 
 
  virtual bool handleKey(int key);
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);

  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);

/*
  float min;
  float max;
  float value;
  float step;
  */
};

////////////////////////////////
class Buffer : public ImageNode
{
  protected: 
  boost::mutex frames_mutex;
  std::deque<cv::Mat> frames;
 
  bool setOut();
  bool addCore(cv::Mat& new_frame, bool restrict_size = true);

  public:

  Buffer(const std::string name); 
  
  bool manualUpdate();
  virtual bool update();

  bool add(cv::Mat& new_frame, bool restrict_size = true);
  virtual bool draw(cv::Point2f ui_offset);
  
  virtual cv::Mat get();
  virtual cv::Mat get(const float fr, int& actual_ind=ind);
  virtual cv::Mat get(int ind, int& actual_ind=ind);

  // TBD get(int ind), negative ind index from last
  
  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);

  virtual bool writeImage();
};

 
////////////////////////////////
// Accessed just like a buffer, but there is no deque, just ordered inputs like Add/Multiply have
// TBD make a base classs with things common between Mux and Buffer held there

// TBD need to think about making every port have an enable will prevent it from getting updated. 
// Here that 'means if a mux input isn't getting used it shouldn't be updated.
// Would this be easier to handle if it wasn't modelled after a Buffer, with separate Tap nodes
// that pull out any input they choose?  The solution is every get call would set the update enable
// to true for the port that it got a copy of, and the update stage will follow those enables and 
// then clear them for the next round
class Mux : public Buffer
{
  public:

  Mux(const std::string name); 
 
  virtual bool update();
  virtual bool handleKey(int key);
};
 
class MuxBuffer : public Buffer
{
  Buffer* selected_buffer;

  public:
  MuxBuffer(const std::string name); 
 
  virtual cv::Mat get(const float fr, int& actual_ind=ind);
  virtual cv::Mat get(int ind, int& actual_ind= ind);

  virtual bool update();
  virtual bool handleKey(int key);
};


};
#endif // ifdef __NODES_H__
