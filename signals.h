#ifndef __SIGNALS_H__
#define __SIGNALS_H__

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

class MiscSignal : public Signal
{
  cv::RNG rng;
  int state;
  public:
  MiscSignal(const std::string name); 
  virtual bool update();
  virtual bool handleKey(int key);
};

/////////////////////////////////
class SigBuffer : public ImageNode
{
  protected:
  std::deque<float> sigs;

  public:
  SigBuffer(const std::string name);
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
  
  float get(const float fr);
  float get(int ind);


  //bool writeSignals();
};

class Trig : public Node
{
  public:
  Trig(const std::string name); 
  virtual bool update();
  //virtual bool handleKey(int key);
};

// Arbitrary inputs, TBD use multiple inheritance 
class SigAdd : public Node
{
  public:
  SigAdd(const std::string name); // : Signal()
  virtual bool update();
  virtual bool handleKey(int key);
};

class SigGreater : public Signal
{
  public:
  SigGreater(const std::string name); 
  virtual bool update();
};

class Mean : public Signal
{
  public:
  Mean(const std::string name); 
  virtual bool update();
};



#ifdef NOT_YET_IMPLEMENTED
/////////////////////////////////

class SigFile : public SigBuffer
{
  public:
  SigFile(const std::string name) {}
  std::string file;
  bool loadBuffer();
  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);
};

///////////////////////////////////////////////////////////
class SigTap : public Signal
{
  public:
  SigTap(const std::string name);// : Signal()
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};

class SigTapInd : public Tap
{
  public:
  SigTapInd(const std::string name) {}// : Signal()
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};


class SigMultiply : public Signal
{
  public:
  Multiply(const std::string name); 
  virtual bool update();
};

// Double inputs
class SigAbsDiff : public Signal
{
  public:
  SigAbsDiff(const std::string name); 
  virtual bool update();
};


class SigFlip : public Signal
{
  public:
  SigFlip(const std::string name);
  virtual bool update();
};
#endif


} // bm
#endif // MISC_NODES
