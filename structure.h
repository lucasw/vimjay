#ifndef __STRUCTURE_H__
#define __STRUCTURE_H__

#include "nodes.h"

//#include <iostream>
//#include <stdio.h>
namespace bm {

class Contour : public ImageNode
{
  public:
  Contour();
  virtual bool update();
};



// TBD an IIR could be generated from a FIR chained to another FIR with an add block at the end
// but it would be nice to be able to capture that inside a single Node- how to correctly handle 
// hierarchical nodes?

} // namespace bm
#endif // __STRUCTURE_H__
