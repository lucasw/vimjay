/*
  
  Copyright 2012 Lucas Walter

     This file is part of Vimjay.

    Vimjay is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Vimjay is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Vimjay.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "output.h"

#include <boost/timer.hpp>
#include <glog/logging.h>

#include <GL/glut.h>

#include "config.h"
#include "utility.h"
#include "opengl.h"

namespace bm {

  OpenGL::OpenGL(const std::string name) :
    ImageNode(name),
    has_setup(false)
  { 
    cv::Mat out; setImage("in",out);

/*    setSignal("decor",1, false, SATURATE, 0, 1); // TBD make a SATURATE_INTEGER, or ROLL_INTEGER type?
    setSignal("mode", 0, false, ROLL, 0, 4);

    setSignal("x", Config::inst()->ui_width, false, SATURATE, 0, 1e6); // TBD FLT_MAX instead of 1e6?
    setSignal("y", 0, false, SATURATE, 0, 1e6);
    setSignal("w", Config::inst()->out_width,  false, SATURATE, 1, 1e6);
    setSignal("h", Config::inst()->out_height, false, SATURATE, 1, 1e6);
    */
  }

  bool OpenGL::setup()
  {
    cv::Size sz = Config::inst()->getImSize();
    int argc = 0;
    char* argv = "ogl";
    glutInit(&argc, &argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA); 
    glutInitWindowSize(sz.width, sz.height);
    glutInitWindowPosition(0,0);
    glutCreateWindow("opengl"); 
    //glutDisplayFunc(display);
    //init();
    //glutMainLoop();

    return true;
  }

  bool OpenGL::update()
  {
    const bool rv = ImageNode::update();
    if (!rv) return false;
    // The threading issues with xwindows for the xwindows calls to be put
    // in the draw call
    return true;
  }

  bool OpenGL::draw(cv::Point2f ui_offset)
  { 
    if (!has_setup) setup();
    has_setup = true;

    boost::timer t1;
   
    cv::Mat in = getImage("in");
    if (in.empty()) {
      in = cv::Mat( Config::inst()->getImSize(), CV_8UC4);
      setImage("in", in);
    }

    {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glBegin(GL_TRIANGLES);
    glVertex3f(-0.5,-0.5,0.0);
    glVertex3f(0.5,0.0,0.0);
    glVertex3f(0.0,0.5,0.0);
    glEnd();

    glutSwapBuffers();
    }

    return ImageNode::draw(ui_offset);
  }

} // namespace bm

