#ifndef __OUTPUT_H__
#define __OUTPUT_H__

#include "nodes.h"

namespace bm {

// TBD allow multiple?
class Output : public ImageNode
{
  GC gc;
  Screen* screen;

  XImage* ximage;

  public:

	Display *display;
  Window win;
  int opcode;
  
  Output();
  
  bool setup(const int width, const int height);
  
  virtual bool update();
  virtual bool draw();
};

} // bm

#endif // __OUTPUT_H__
