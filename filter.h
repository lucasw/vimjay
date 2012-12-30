#ifndef __FILTER_H__
#define __FILTER_H__

#include "nodes.h"
#include "modify.h"

//#include <iostream>
//#include <stdio.h>
namespace bm {

class FilterFIR : public Buffer
{
  
  public:

  /// filter coefficients
  std::vector<float> xi;

  FilterFIR(const std::string name);

  void setup(const std::vector<float> new_xi);
  virtual bool update();
 
  virtual bool handleKey(int key);

};

class Sobel : public ImageNode
{
  public:
  Sobel(const std::string name);
  virtual bool update();
};

class Laplacian : public ImageNode
{
  public:
  Laplacian(const std::string name);
  virtual bool update();
};

class GaussianBlur : public ImageNode
{
  public:
  GaussianBlur(const std::string name);
  virtual bool update();
};

class MedianBlur : public ImageNode
{
  public:
  MedianBlur(const std::string name);
  virtual bool update();
};

class BilateralFilter : public ImageNode
{
  public:
  BilateralFilter(const std::string name);
  virtual bool update();
};

class InPaint : public ImageNode
{
  public:
  InPaint(const std::string name);
  virtual bool update();
};



class MorphologyEx : public ImageNode
{
  public:
  MorphologyEx(const std::string name);
  virtual bool update();
};

class OpticalFlow : public Remap
{
  // a CV_32F image, so not suitable for use in a signal/connector
  cv::Mat flow;
  cv::Mat flow_reverse;

  public:
  OpticalFlow(const std::string name);
  virtual bool update();
};


// TBD an IIR could be generated from a FIR chained to another FIR with an add block at the end
// but it would be nice to be able to capture that inside a single Node- how to correctly handle 
// hierarchical nodes?

} // namespace bm
#endif // __FILTER_H__
