#ifndef __GENERATE_H__
#define __FILTER_H__

#include "nodes.h"

/**
Nodes which generate images
*/

//#include <iostream>
//#include <stdio.h>
namespace bm {

class Bezier : public ImageNode
{
  public:
  Bezier();
  virtual bool update();
};

} // namespace bm
#endif // __FILTER_H__
