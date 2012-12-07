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

#include "signals.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>

#include <deque>

#include "config.h"

using namespace cv;
using namespace std;

namespace bm {
//////////////////////////////////////////////////
  MiscSignal::MiscSignal(const std::string name) : Signal(name)
  {
    vcol = cv::Scalar(0,90,255);
  }
  
  //void MiscSignal::setup(const float new_step, const float offset, const float min, const float max) 
 // {
 //   Signal::setup(new_step, offset, min, max);
  //}

  bool MiscSignal::handleKey(int key)
  {
    bool valid_key = Signal::handleKey(key);
    if (valid_key) return true;
    
    //valid_key = true;

    // TBD 
    if (valid_key) setDirty();
    
    return valid_key;
  }

  bool MiscSignal::update()
  {
    // don't call Signal::update because it will contradict this update
    if (!Node::update()) return false;

    float step = getSignal("step");
    float min = getSignal("min");
    float max = getSignal("max");
    float value = getSignal("value");
 
    if (max < min) {
      max = min;
      setSignal("max", max);
    }

    string port = "mode";
    int mode = getSignal(port);
    
    Connector* con = NULL;
    string src_port;
    getInputPort(SIGNAL, port, con, src_port);
    
    float span = max - min;

    if (mode == 0) {
      con->description = "sawtooth";
      value += step;
      if (span == 0) {
        value = min;
      } else {
        while (value > max) {
          value -= (span);
        }
        while (value < min) {
          value += (span);
        }
      }

    } else if (mode == 1) {
      con->description = "ramp_saturate";
      value += step;
      if (value > max) value = max;
      if (value < min) value = min;

    } else if (mode == 2) {
      con->description = "triangle";
      value += step;
      if (value > max) {
        step = -fabs(step);
        value = max;
      }
      if (value < min) {
        step = fabs(step);
        value = min;
      }

    } else if (mode == 3) {
      con->description = "toggle";
      if (value < max) {
        value = max;
      }
      if (value > min) {
        value = min;
      }

    } else if (mode == 4) {
      con->description = "rnd_uniform";
      value = rng.uniform( min, max );
      
    } else if (mode == 5) {
      con->description = "rnd_gaussian";
      const float rnd = rng.gaussian( max - min );
      value = rnd + min;
    }

    setSignal("step", step);
    setSignal("value", value);

    setDirty();

    VLOG(3) << "Signal " << name << " " << value;
    return true;
  }

  // TBD need control to set value to max or min
 
  ///////////////////////////////////////////////////
  Trig::Trig(const string name) :
      Node(name)
  { 
    setSignal("in",0);
    setSignal("radius",1);
    setSignal("rad_deg_nrm",2);
    setSignal("cos",0, true);
    setSignal("sin",0, true);
    setSignal("tan",0, true);
  }

  bool Trig::update()
  {
    if (!Node::update()) return false;
    
    const int rad_deg_nrm = getSignal("rad_deg_nrm");
    float in = getSignal("in");
    const float radius = getSignal("radius"); 
    // convert deg to radians
    if (rad_deg_nrm == 1) in *= M_PI/180.0;
    // convert from normalized radians to radians
    if (rad_deg_nrm == 2) in *= M_PI; 
    
    setSignal("cos", radius*cos(in));
    setSignal("sin", radius*sin(in));
    setSignal("tan", radius*tan(in));

    return true;
  }
  /////////////////////////////////////////////////////////////////////////////
  SigBuffer::SigBuffer(const std::string name) :
      ImageNode(name)
  {
    setSignal("in", 0);
    
    VLOG(1) <<" setting out image";
    cv::Mat tmp;
    cv::Size sz = Config::inst()->getImSize();
    tmp = cv::Mat(sz, MAT_FORMAT_C3, cv::Scalar(0));
    setImage("out", tmp);
    setSignal("out", 0);
    setSigBuf("out", true);

    setSignal("min", 0);
    setSignal("max", 0);
    setSignal("max_size", 100);
    setSignal("cur_size", 0);
   
  }

  bool SigBuffer::update()
  {
    const bool rv = Node::update();
    if (!rv) return false;

    int max_size = getSignal("max_size");
    if (max_size < 0) max_size = 0;

    const float new_sig = getSignal("in");
    // TBD wait to be dirty, or sample signal anyway?  Control behaviour with parameter?
    // probably should check to see if it is dirty
    sigs.push_back(new_sig);

    while (sigs.size() > max_size) sigs.pop_front(); 
    
    setSignal("cur_size", sigs.size());
   
    setSignal("out", sigs[0]);

    setDirty();

    return true;
  }

  bool SigBuffer::draw(cv::Point2f ui_offset)
  {
    cv::Mat vis = getImage("out");
    if (vis.empty()) {
      LOG(WARNING) << "out is empty";
      cv::Size sz = Config::inst()->getImSize();
      vis = cv::Mat(sz, MAT_FORMAT_C3, cv::Scalar(0));
    }

    // update the vis image- TBD add option to disable this?
    if (isDirty(this, 24)) {
    
      // TBD error check the mat
      vis = cv::Scalar(0);

      float new_min = 1e6;
      float new_max = -1e6;
      const float min = getSignal("min");
      const float max = getSignal("max");

      float sc = fabs(max);
      if (fabs(min) > sc) sc = fabs(min);
      sc *= 2.1;
    
      float div = (float)sigs.size()/(float)vis.cols;

      int inc = 1;
      if (div > 1.0) inc = (int) div;
      
      VLOG(5) <<sigs.size()<<":"<< div << " " << inc << " " << sc;

      for (int i = 0; i < (int)sigs.size() - 1; i += inc) {
        const float val = sigs[i];
        const float val2 = sigs[i+1];
        if (val > new_max) new_max = val;
        if (val < new_min) new_min = val;

        cv::line( vis,
          cv::Point2f((float)(i)/div,     vis.rows * (0.5 + val/sc)),
          cv::Point2f((float)(i + 1)/div, vis.rows * (0.5 + val2/sc)),
          cv::Scalar(255,255,255), (div + 1), 4);
      }
     
      // an external signal will override, but no way for manual input to override
      setSignal("min", new_min);
      setSignal("max", new_max);
    
      setImage("out", vis);
    }

    return ImageNode::draw(ui_offset);
  }

  //bool SigBuffer::writeSignals();

  // not the same as the inherited get on purpose
  // many callers per time step could be calling this
  float SigBuffer::get(const float fr) 
  {
    const int ind = (int)(fr * (float)sigs.size());
    //if (fr < 0) {
    //  ind = frames.size() - ind;
    //}
    
    return get(ind);
  }

  float SigBuffer::get(int ind)
  {
    if (sigs.size() < 1) {
      VLOG(1) << "no sigs returning 0";
      return 0;
    }
    //if (ind > sigs.size() - 1) ind = sigs.size() - 1;
    //if (ind < 0) ind = 0;
    ind %= sigs.size();
  
    VLOG(2) << name << " ind " << ind;

    //VLOG_EVERY_N(1,10) 
    //LOG_EVERY_N(INFO, 10) << ind << " " << sigs.size();
    return sigs[ind];
  }

  
// Sig add is similar in structure to the image multiply, 
// so it has pairs of signal inputs that are each multiplied together
// but then each set is summed
SigAdd::SigAdd(const std::string name) :
    Node(name)
{
  setSignal("out",0, true);
  setSignal("mula0",1);
  setSignal("mulb0",0);
}

bool SigAdd::update()
{
  if (!Node::update()) return false;

  if (!isDirty(this, 35)) {
    VLOG(1) << name << " not dirty ";
    return true;
  }

  float sum = 0;

  for (int i = 0 ; i < ports.size(); i++) {
    if (ports[i]->type != SIGNAL) continue;
    const string port_a = ports[i]->name;

    if (port_a.substr(0,4) != "mula") {
      VLOG(2) << name << " : " << port_a.substr(0,4) << " " << port_a;
      continue;
    }
  
    const string port_b = "mulb" + port_a.substr(4);
    
    const float a = getSignal(port_a);
    const float b = getSignal(port_b);

    VLOG(2) << name <<  " " << CLTXT << port_a << " " << port_b << " " 
        << CLVAL << sum << " + " << a << " * " << b << CLNRM;
    sum += getSignal(port_a) * getSignal(port_b);  
    
  }
  VLOG(2) << name << " " << sum;

  setSignal("out", sum);

  // TBD The dirtiness of the output signal takes care of this?
  setDirty();
  isDirty(this,35);

  return true;

}

bool SigAdd::handleKey(int key)
{
  bool valid_key = Node::handleKey(key);
  if (valid_key) return true;

  valid_key = true;
  if (key == '[') {

    // add an input addition port, TBD move to function
    int add_num = 0;
    for (int i = 0; i < ports.size(); i++) {
      if (ports[i]->type != SIGNAL) continue;
      const string port = ports[i]->name;

      if (port.substr(0,4) != "mula") {
        VLOG(1) << name << " : " << port.substr(0,4) << " " << port;
        continue;
      }
      add_num++;
    }

    // add a new addition port
    const string port_a = "mula" + boost::lexical_cast<string>(add_num);
    const string port_b = "mulb" + boost::lexical_cast<string>(add_num);
    setSignal(port_a, 1);
    setSignal(port_b, 0);

    // TBD make a way to delete a port
  } else {
    valid_key = false;
  }

  // TBD 
  if (valid_key) setDirty();

  return valid_key;
}


  

} //bm

