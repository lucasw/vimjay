#ifndef __FILTER_H__
#define __FILTER_H__

#include "nodes.h"

//#include <iostream>
//#include <stdio.h>
namespace bm {

class FilterFIR : public Buffer
{
  
  public:

  /// filter coefficients
  std::vector<float> xi;

  FilterFIR();

  void setup(const std::vector<float> new_xi);
  virtual bool update();
  
  virtual bool save(cv::FileStorage& fs);
 

};

// TBD an IIR could be generated from a FIR chained to another FIR with an add block at the end
// but it would be nice to be able to capture that inside a single Node- how to correctly handle 
// hierarchical nodes?

} // namespace bm
#endif // __FILTER_H__
