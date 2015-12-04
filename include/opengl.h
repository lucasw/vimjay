#ifndef VOPENGL_H
#define VOPENGL_H

#include "nodes.h"
#include "misc_nodes.h"

#include <GL/gl.h>
#include <string>

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

#endif  // VOPENGL_H

