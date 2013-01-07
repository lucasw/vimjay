#ifndef __STRUCTURE_H__
#define __STRUCTURE_H__

#include "nodes.h"

//#include <iostream>
//#include <stdio.h>
namespace bm {

class Contour : public ImageNode
{
  protected:
  std::vector<std::vector<cv::Point> > contours0;
  std::vector<cv::Vec4i> hierarchy;
  
  public:
  Contour(const std::string name);
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
  ContourFlip(const std::string name);
  virtual bool update();
};

// TBD an IIR could be generated from a FIR chained to another FIR with an add block at the end
// but it would be nice to be able to capture that inside a single Node- how to correctly handle 
// hierarchical nodes?

} // namespace bm
#endif // __STRUCTURE_H__
