/** Copyright 2012 Lucas Walter */
#ifndef VIMJAY_SIGNALS_H
#define VIMJAY_SIGNALS_H

#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// #include <random>
#include <deque>
#include <map>
#include <string>

#include "vimjay/nodes.h"

namespace bm
{

class MiscSignal : public Signal
{
  cv::RNG rng;
  int state;
public:
  explicit MiscSignal(const std::string name);
  virtual void init();
  virtual bool update();
  virtual bool handleKey(int key);
};

/////////////////////////////////
class SigBuffer : public ImageNode
{
protected:
  std::deque<float> sigs;
  bool setOut();
  boost::mutex sigs_mutex;
  bool last_get_was_fr;

public:
  explicit SigBuffer(const std::string name);
  virtual void init();
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);

  float getFr(const float fr);
  float getInd(int& ind);

  // bool writeSignals();
};

class SigBufferXY : public SigBuffer
{
public:
  explicit SigBufferXY(const std::string name);
  virtual void init();
  virtual bool update();
};

class Trig : public Node
{
public:
  explicit Trig(const std::string name);
  virtual void init();
  virtual bool update();
  // virtual bool handleKey(int key);
};

// Arbitrary inputs, TBD use multiple inheritance
class SigAdd : public Node
{
public:
  explicit SigAdd(const std::string name);  // : Signal()
  virtual void init();
  virtual bool update();
  virtual bool handleKey(int key);
};

class SigGreater : public Signal
{
public:
  explicit SigGreater(const std::string name);
  virtual void init();
  virtual bool update();
};

class Mean : public Signal
{
public:
  explicit Mean(const std::string name);
  virtual void init();
  virtual bool update();
};

class SigADSR : public ImageNode
{
public:
  explicit SigADSR(const std::string name);
  virtual void init();
  virtual bool update();
};

// TBD could be image node, light up
// blocks to show which signal inputs
// are active
class SigToInd : public Signal
{
public:
  explicit SigToInd(const std::string name);
  virtual void init();
  virtual bool update();
};

#ifdef NOT_YET_IMPLEMENTED
/////////////////////////////////

class SigFile : public SigBuffer
{
public:
  explicit SigFile(const std::string name) {}
  virtual void init();
  std::string file;
  bool loadBuffer();
  virtual bool load(cv::FileNodeIterator nd);
  virtual bool save(cv::FileStorage& fs);
};

///////////////////////////////////////////////////////////
class SigTap : public Signal
{
public:
  explicit SigTap(const std::string name);  // : Signal()
  virtual void init();
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};

class SigTapInd : public Tap
{
public:
  explicit SigTapInd(const std::string name) {}  // : Signal()
  virtual void init();
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};


class SigMultiply : public Signal
{
public:
  explicit Multiply(const std::string name);
  virtual void init();
  virtual bool update();
};

// Double inputs
class SigAbsDiff : public Signal
{
public:
  explicit SigAbsDiff(const std::string name);
  virtual void init();
  virtual bool update();
};


class SigFlip : public Signal
{
public:
  explicit SigFlip(const std::string name);
  virtual void init();
  virtual bool update();
};
#endif

}  // namespace bm
#endif  // VIMJAY_SIGNALS_H
