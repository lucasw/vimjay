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

#include <GL/glew.h>
#include <GL/glext.h>
#include <GL/glut.h>

#include "opengl.h"

#include <boost/timer.hpp>
#include <glog/logging.h>


#include "config.h"
#include "utility.h"

namespace bm {

  OpenGL::OpenGL(const std::string name) :
    ImageNode(name),
    has_setup(false)
  { 
    cv::Mat out; setImage("in",out);

    setSignal("x1", -1.0);
    setSignal("y1", -1.0);
    setSignal("z1",  0.0);
    setSignal("u1",  0.0);
    setSignal("v1",  0.0);
/*    setSignal("decor",1, false, SATURATE, 0, 1); // TBD make a SATURATE_INTEGER, or ROLL_INTEGER type?
    setSignal("mode", 0, false, ROLL, 0, 4);

    setSignal("x", Config::inst()->ui_width, false, SATURATE, 0, 1e6); // TBD FLT_MAX instead of 1e6?
    setSignal("y", 0, false, SATURATE, 0, 1e6);
    setSignal("w", Config::inst()->out_width,  false, SATURATE, 1, 1e6);
    setSignal("h", Config::inst()->out_height, false, SATURATE, 1, 1e6);
    */
  }

  void makeTexture(GLuint& textureId, const int width, const int height) 
  {
    glGenTextures(1, &textureId);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE); // automatic mipmap

  }


  bool OpenGL::setup()
  {
    cv::Size sz = Config::inst()->getImSize();
    int argc = 0;
    char* argv = (char*)"ogl";
    
    // TBD only want to render to texture, is creating this window necessary?
    glutInit(&argc, &argv);
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA); 
    //glutInitWindowSize(5, 5);
    glutInitWindowSize(sz.width, sz.height);
    glutInitWindowPosition(Config::inst()->ui_width,0);
    glutCreateWindow("opengl"); 


    {
    glewInit();
    
    if(!glGenFramebuffers) {
      LOG(ERROR) << "can't use glGenFramebuffers";
      return false;
    }
    // create a framebuffer object
    glGenFramebuffers(1, &fboId);
    glBindFramebuffer(GL_FRAMEBUFFER, fboId);
    
    // create a renderbuffer object to store depth info
    glGenRenderbuffers(1, &rboId);
    glBindRenderbuffer(GL_RENDERBUFFER, rboId);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT,
        sz.width, sz.height);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);


    // create a texture object
    GLuint textureId;
    makeTexture(textureId, sz.width, sz.height);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, sz.width, sz.height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    
    // attach the texture to FBO color attachment point
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D, textureId, 0);

    // attach the renderbuffer to depth attachment point
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER, rboId);

    }
    // check FBO status
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    bool fboUsed = true;

    if(status != GL_FRAMEBUFFER_COMPLETE) {
      LOG(ERROR)<< "fbo incomplete";
      fboUsed = false;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    
    // setup input texture
    makeTexture(input_tex, sz.width, sz.height);

    cv::Mat tmp = cv::Mat( Config::inst()->getImSize(), CV_8UC3);
    tmp = cv::Scalar(255, 0, 0,0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, tmp.cols, tmp.rows, 0, 
        GL_RGB, GL_UNSIGNED_BYTE, tmp.data);

    glEnable(GL_TEXTURE_2D);
    //glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );
    glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL );
    
    glBindTexture(GL_TEXTURE_2D, 0);

    return true;
  }

  bool OpenGL::update()
  {
    const bool rv = Node::update();
    if (!rv) return false;
    // The threading issues with xwindows for the xwindows calls to be put
    // in the draw call

    if (!isDirty(this,21)) { return true;}

    if (!has_setup) setup();
    has_setup = true;

    cv::Mat in = getImage("in").clone();
    if (in.empty()) {
      in = cv::Mat( Config::inst()->getImSize(), CV_8UC4);
      setImage("in", in);
    }
    
    cv::Mat in_flipped;
    cv::flip(in, in_flipped, 0);

    
    {
    glBindFramebuffer(GL_FRAMEBUFFER, fboId);
    glPushMatrix();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float x1 = getSignal("x1");
    float y1 = getSignal("y1");
    float z1 = getSignal("z1");
    float u1 = getSignal("u1");
    float v1 = getSignal("v1");

    glBindTexture(GL_TEXTURE_2D, input_tex);
    
    cv::Mat in3 = cv::Mat(in.size(), CV_8UC3);;
    int ch[] = {0,2, 1,1, 2,0};
    cv::mixChannels(&in, 1, &in3, 1, ch, 3 );

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, in.cols, in.rows, 0, 
        GL_RGB, GL_UNSIGNED_BYTE, in3.data);
    
    glBegin(GL_QUADS);
    
    glTexCoord2f(u1, v1);
    glVertex3f(x1,y1,z1);
    
    glTexCoord2f(1.0, 0.0);
    glVertex3f(1.0,-1.0,0.0);
    
    glTexCoord2f(1.0, 1.0);
    glVertex3f(1.0,1.0,0.0);

    glTexCoord2f(0.0, 1.0);
    glVertex3f(-1.0,1.0,0.0);
    
    glEnd();
    
    glPopMatrix();

    glutSwapBuffers();

    }
    
    {
    cv::Mat out = cv::Mat( Config::inst()->getImSize(), CV_8UC4);
    // now extract image back to opencv
    //use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_PACK_ALIGNMENT, (out.step & 3) ? 1 : 4);

    //set length of one complete row in destination data (doesn't need to equal img.cols)
    glPixelStorei(GL_PACK_ROW_LENGTH, out.step/out.elemSize());

    glReadPixels(0, 0, out.cols, out.rows, GL_BGRA, GL_UNSIGNED_BYTE, out.data);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    
    cv::Mat out_flipped;
    cv::flip(out, out_flipped, 0);

    setImage("out", out_flipped); 
    }


    return true;
  }

  bool OpenGL::draw(cv::Point2f ui_offset)
  { 

    boost::timer t1;
   
    
    return ImageNode::draw(ui_offset);
  }

} // namespace bm

