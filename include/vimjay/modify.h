/**
  * Copyright 2014 Lucas Walter
  */
#ifndef MODIFY_H
#define MODIFY_H

#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// #include <random>
#include <deque>
#include <map>
#include <string>
#include <vector>

#include "vimjay/nodes.h"

namespace bm
{
class Rot2D : public ImageNode
{
public:
  explicit Rot2D(const std::string name);
  virtual void init();
  virtual bool update();
};

class Kaleid : public ImageNode
{
protected:
  cv::Mat base_x;
  cv::Mat base_y;

public:
  explicit Kaleid(const std::string name);
  virtual void init();
  virtual bool update();
};


class Undistort : public ImageNode
{
public:
  explicit Undistort(const std::string name);
  virtual void init();
  virtual bool update();
};

class Remap : public ImageNode
{
protected:
// don't expose these to ui with set/getSignals
  cv::Mat base_x;
  cv::Mat base_y;
  cv::Mat base_xy;

public:
  explicit Remap(const std::string name);
  virtual void init();
  virtual bool update();
};
///////////////////////////////////////////////////////////
class Tap : public ImageNode
{
public:
  bool changed;
  // float value;

  explicit Tap(const std::string name);  // : ImageNode()
  virtual void init();

  void setup(
    boost::shared_ptr<Signal> new_signal = boost::shared_ptr<Signal>(),
    boost::shared_ptr<Buffer> new_buffer = boost::shared_ptr<Buffer>());

  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};

class TapInd : public Tap
{
public:
  explicit TapInd(const std::string name) : Tap(name) {}
  virtual void init();

  // TBD make an sval?
  // int ind;
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};

// Arbitrary inputs
class Add : public ImageNode
{
public:
  explicit Add(const std::string name);
  virtual void init();
  // TBD could require pair be passed in to enforce size
  // TBD get rid of this?
  void setup(
    std::vector<boost::shared_ptr<ImageNode> > np,
    std::vector<float> nf);
  virtual bool update();
  virtual bool handleKey(int key);
};

// Only 2 inputs
class AddMasked : public ImageNode
{
public:
  explicit AddMasked(const std::string name);
  virtual void init();
  virtual bool update();
  // virtual bool handleKey(int key);
};

class Multiply : public ImageNode
{
public:
  explicit Multiply(const std::string name);
  virtual void init();
  virtual bool update();
};

// Double inputs
class AbsDiff : public ImageNode
{
public:
  explicit AbsDiff(const std::string name);
  virtual void init();
  virtual bool update();
};

class Max : public ImageNode
{
public:
  explicit Max(const std::string name);
  virtual void init();
  virtual bool update();
};

class Greater : public ImageNode
{
public:
  explicit Greater(const std::string name);
  virtual void init();
  virtual bool update();
};

// Single inputs
class Resize : public ImageNode
{
public:
  explicit Resize(const std::string name);
  virtual void init();
  virtual bool update();
};

class Flip : public ImageNode
{
public:
  explicit Flip(const std::string name);
  virtual void init();
  virtual bool update();
};

class EqualizeHist : public ImageNode
{
public:
  explicit EqualizeHist(const std::string name);
  virtual void init();
  virtual bool update();
};

class Normalize : public ImageNode
{
public:
  explicit Normalize(const std::string name);
  virtual void init();
  virtual bool update();
};

class Distance : public ImageNode
{
public:
  explicit Distance(const std::string name);
  virtual void init();
  virtual bool update();
};

class DistanceFlip : public ImageNode
{
public:
  explicit DistanceFlip(const std::string name);
  virtual void init();
  virtual bool update();
};

class FloodFill : public ImageNode
{
public:
  explicit FloodFill(const std::string name);
  virtual void init();
  virtual bool update();
};


}  // namespace bm
#endif  // MODIFY_H
