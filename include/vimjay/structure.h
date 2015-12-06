/** Copyright 2012 Lucas Walter */
#ifndef VIMJAY_STRUCTURE_H
#define VIMJAY_STRUCTURE_H

#include "vimjay/nodes.h"

#include <string>
#include <vector>

// #include <iostream>
// #include <stdio.h>
namespace bm
{

class Contour : public ImageNode
{
protected:
  std::vector<std::vector<cv::Point> > contours0;
  std::vector<cv::Vec4i> hierarchy;

public:
  explicit Contour(const std::string name);
  virtual void init();
  virtual bool update();
};

class ContourFlip : public Contour
{
protected:
  cv::Mat base_x;
  cv::Mat base_y;

  cv::Mat off_x;
  cv::Mat off_y;

  cv::Mat dist_xy16, dist_int;

public:
  explicit ContourFlip(const std::string name);
  virtual void init();
  virtual bool update();
};

// TBD an IIR could be generated from a FIR chained to another FIR with an add block at the end
// but it would be nice to be able to capture that inside a single Node- how to correctly handle
// hierarchical nodes?

}  // namespace bm
#endif  // VIMJAY_STRUCTURE_H
