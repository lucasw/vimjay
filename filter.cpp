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
  bool rv = Buffer::update();
  if (!rv) return false;

  cv::Mat out;

  for (int i = 0; i < xi.size() && i < frames.size(); i++) {

    cv::Mat tmp = frames[frames.size() - i - 1] * xi[i];  
    if (i == 0) {
      out = tmp;
    } else {
      out += tmp;
    }
  }

  setImage("out", out);

  return true;
}
 
  bool FilterFIR::load(cv::FileNodeIterator nd)
  {
    Buffer::load(nd);

    for (int i = 0; i < (*nd)["xi"].size(); i++) {
      float new_coeff;
      (*nd)["xi"][i] >> new_coeff;
      xi.push_back(new_coeff);
    }
  }

bool FilterFIR::save(cv::FileStorage& fs) 
{
  Buffer::save(fs);

  fs << "xi" << "[:";
  for (int j = 0; j < xi.size(); j++) {
    fs << xi[j];
  }
  fs << "]";
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
  if (ksize == 2) ksize= 3;
  if (ksize == 3) ksize= 5;
  if (ksize > 4) ksize = 7;
  if (xorder < 1) xorder = 1;
  if (xorder > 2) xorder = 2;
  if (yorder < 1) yorder = 1;
  if (yorder > 2) yorder = 2;
  if (scale < 0.001) scale = 0.001;

  setSignal("ksize", ksize);
  setSignal("xorder", xorder);
  setSignal("yorder", yorder);
  setSignal("scale", scale);

  cv::Mat out;
  cv::Sobel(in, out, in.depth(), 
      xorder,
      yorder,
      ksize, 
      scale
      );
  setImage("out", out);

  return true;
}


} // namespace bm

