#ifndef __VOPENGL_H__
#define __VOPENGL_H__

#include "nodes.h"
#include "misc_nodes.h"

//#include <iostream>
//#include <stdio.h>
namespace bm {

class OpenGL : public ImageNode
{
  bool has_setup;

  public:
  OpenGL(const std::string name);
  
  bool setup();
  
  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);

};

}

#endif // __VOPENGL_H__

