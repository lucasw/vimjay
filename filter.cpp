/*
  
  Copyright 2012 Lucas Walter

     This file is part of Camthing.

    Camthing is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Camthing is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Camthing.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "filter.h"

#include <boost/lexical_cast.hpp>

#include <glog/logging.h>

// filter Node objects
namespace bm {

FilterFIR::FilterFIR()
{

}

void FilterFIR::setup(const std::vector<float> new_xi)
{
  xi = new_xi;
  max_size = new_xi.size();
}

bool FilterFIR::update()
{
  bool rv = Node::update();
  if (!rv) return false;
  
  if (!isDirty(this,22)) { return true;}
  
  manualUpdate();

  cv::Mat out;

  int addnum =  0;
  for (int i = 0; i < ports.size(); i++) {
    if (ports[i]->type != SIGNAL) continue;
    const std::string port = ports[i]->name;

    if (port.substr(0,2) != "xi") continue;
    float val = getSignal(port);

    cv::Mat tmp = frames[frames.size() - addnum - 1];  

    if (addnum == 0) 
      out = tmp * val;
    else {
      if (val > 0) {
        out += tmp * val;
      } else 
        out -= tmp * -val;
    }
    
    addnum++;
    
    if (addnum >= frames.size()) break;
  }

  setImage("out", out);

  return true;
}

bool FilterFIR::handleKey(int key)
{
  bool valid_key = Buffer::handleKey(key);
  if (valid_key) return true;

  valid_key = true;
  if (key == '[') {

    // add an input addition port, TBD move to function
    int add_num = 0;
    for (int i = 0; i < ports.size(); i++) {
      if (ports[i]->type != SIGNAL) continue;
      const std::string port = ports[i]->name;

      if (port.substr(0,2) != "xi") {
        VLOG(1) << name << " : " << port.substr(0,2) << " " << port;
        continue;
      }
      add_num++;
    }

    setSignal("max_size", add_num+1);

    // add a new addition port
    const std::string port = "xi" + boost::lexical_cast<std::string>(add_num);
    setInputPort(SIGNAL, port, NULL, "value"); // this allows other signals to connect to replace nf

    // TBD make a way to delete a port
  } else {
    valid_key = false;
  }

  // TBD 
  if (valid_key) setDirty();

  return valid_key;
}


Sobel::Sobel()
{
  cv::Mat tmp;
  setImage("in", tmp);
  setSignal("xorder",1);
  setSignal("yorder",1);
  setSignal("ksize",3);
  setSignal("scale",0.8);
}

bool Sobel::update()
{
  //if (!ImageNode::update()) return false;
  if (!Node::update()) return false;

  cv::Mat in = getImage("in");
  if (in.empty()) {
    VLOG(2) << name << " in is empty";
    return false;
  }

  int ksize = getSignal("ksize");
  int xorder = getSignal("xorder"); 
  int yorder = getSignal("yorder"); 
  float scale = getSignal("scale");

  if (ksize < 1) ksize = 1;
  if (ksize > 4) ksize = 4;
  if (xorder < 1) xorder = 1;
  if (xorder > 2) xorder = 2;
  if (yorder < 1) yorder = 1;
  if (yorder > 2) yorder = 2;
  if (scale < 0.001) scale = 0.001;

  
  setSignal("ksize", ksize);
  setSignal("xorder", xorder);
  setSignal("yorder", yorder);
  setSignal("scale", scale);

  int real_ksize = 1;
  if (ksize == 2) real_ksize = 3;
  if (ksize == 3) real_ksize = 5;
  if (ksize == 4) real_ksize = 7;

  cv::Mat out;
  cv::Sobel(in, out, in.depth(), 
      xorder,
      yorder,
      real_ksize, 
      scale
      );
  setImage("out", out);

  return true;
}

////////////////////////////////////////
  GaussianBlur::GaussianBlur() 
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("k_width",2);
    setSignal("k_height",2);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool GaussianBlur::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
    
    cv::Mat in = getImage("in");

    if (in.empty()) return false;

    int k_width = abs(getSignal("k_width"));
    int k_height = abs(getSignal("k_height"));
     
    cv::Size ksize = cv::Size(k_width*2 + 1, k_height*2+1);

    cv::Mat out = cv::Mat(in.size(), in.type());

    cv::GaussianBlur(in, out, ksize, 0, 0, cv::BORDER_REFLECT);
    
    setImage("out", out);
    return true;
  }

} // namespace bm

