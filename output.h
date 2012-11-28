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
  
  Output(const std::string name);
  
  bool setup(const int width, const int height);
  
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};

} // bm

#endif // __OUTPUT_H__
