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

class Saw : public Signal
{
  public:
  Saw(); // : Signal()
  void setup(const float new_step=0.01, const float offset=0.0, const float min =0.0, const float max=1.0); 
  virtual bool update();
  virtual bool handleKey(int key);
};

class Random : public Signal
{
  //TBD use opencv RNG instead
  std::random_device rd;
  std::uniform_real_distribution<> dis;
  std::mt19937 gen;

  public:
  Random(); // : Signal()
  virtual bool update();
  //virtual bool handleKey(int key);
};

#ifdef NOT_YET_IMPLEMENTED
/////////////////////////////////
class SigBuffer : public Signal
{

}

class SigFile : public SigBuffer
{
  public:
  SigFile() {}
  std::string file;
  bool loadBuffer();
  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);
};

///////////////////////////////////////////////////////////
class SigTap : public Signal
{
  public:
  SigTap();// : Signal()
  virtual bool update();
  virtual bool draw();
};

class SigTapInd : public Tap
{
  public:
  SigTapInd() {}// : Signal()
  virtual bool update();
  virtual bool draw();
};

// Arbitrary inputs 
class SigAdd : public Signal
{
  public:
  Add(); // : Signal()
  virtual bool update();
  virtual bool handleKey(int key);
};

class SigMultiply : public Signal
{
  public:
  Multiply(); 
  virtual bool update();
};

// Double inputs
class SigAbsDiff : public Signal
{
  public:
  SigAbsDiff(); 
  virtual bool update();
};

class SigGreater : public Signal
{
  public:
  SigGreater(); 
  virtual bool update();
};

class SigFlip : public Signal
{
  public:
  SigFlip();
  virtual bool update();
};
#endif


} // bm
#endif // MISC_NODES
