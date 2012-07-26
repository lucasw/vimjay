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

};

} // namespace bm
#endif // __FILTER_H__
