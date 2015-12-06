/**
  * Copyright 2014 Lucas Walter
  */
#ifndef FILTER_H
#define FILTER_H

#include "vimjay/nodes.h"
#include "vimjay/modify.h"

#include <string>
#include <vector>

// #include <iostream>
// #include <stdio.h>
namespace bm
{
class FilterFIR : public Buffer
{
public:
  /// filter coefficients
  std::vector<float> xi;

  explicit FilterFIR(const std::string name);
  virtual void init();
  void setup(const std::vector<float> new_xi);
  virtual bool update();

  virtual bool handleKey(int key);
};

class Sobel : public ImageNode
{
public:
  explicit Sobel(const std::string name);
  virtual void init();
  virtual bool update();
};

class Laplacian : public ImageNode
{
public:
  explicit Laplacian(const std::string name);
  virtual void init();
  virtual bool update();
};

class GaussianBlur : public ImageNode
{
public:
  explicit GaussianBlur(const std::string name);
  virtual void init();
  virtual bool update();
};

class MedianBlur : public ImageNode
{
public:
  explicit MedianBlur(const std::string name);
  virtual void init();
  virtual bool update();
};

class BilateralFilter : public ImageNode
{
public:
  explicit BilateralFilter(const std::string name);
  virtual void init();
  virtual bool update();
};

class InPaint : public ImageNode
{
public:
  explicit InPaint(const std::string name);
  virtual void init();
  virtual bool update();
};



class MorphologyEx : public ImageNode
{
public:
  explicit MorphologyEx(const std::string name);
  virtual void init();
  virtual bool update();
};

class OpticalFlow : public Remap
{
  // a CV_32F image, so not suitable for use in a signal/connector
  cv::Mat flow;
  cv::Mat flow_reverse;

public:
  explicit OpticalFlow(const std::string name);
  virtual void init();
  virtual bool update();
};


// TBD an IIR could be generated from a FIR chained to another FIR with an add block at the end
// but it would be nice to be able to capture that inside a single Node- how to correctly handle
// hierarchical nodes?

}  // namespace bm
#endif  // VIMJAY_FILTER_H
