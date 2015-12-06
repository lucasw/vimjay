/** Copyright 2012 Lucas Walter */
#ifndef VIMJAY_OPENGL_H
#define VIMJAY_OPENGL_H

#include <GL/gl.h>
#include <string>

#include "vimjay/nodes.h"
#include "vimjay/misc_nodes.h"

// #include <iostream>
// #include <stdio.h>
namespace bm
{
class OpenGL : public ImageNode
{
  bool has_setup;

  GLuint fboId;
  GLuint rboId;

  // the input texture
  GLuint input_tex;

public:
  explicit OpenGL(const std::string name);
  virtual void init();

  bool setup();

  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};
}  // namespace bm

#endif  // VIMJAY_OPENGL_H

