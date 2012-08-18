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

  cv::Mat new_out;

  for (int i = 0; i < xi.size() && i < frames.size(); i++) {

    cv::Mat tmp = frames[frames.size() - i - 1] * xi[i];  
    if (i == 0) {
      new_out = tmp;
    } else {
      new_out += tmp;
    }
  }

  out = new_out;

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
  inputs["ImageNode"]["image"] = NULL;
}

bool Sobel::update()
{
  if (!ImageNode::update()) return false;
 
  if (out.empty()) {
    VLOG(2) << name << " out is empty";
    return false;
  }

  cv::Mat tmp;
  cv::Sobel(out, tmp, out.depth(), 1, 1, 3, 8);
  out = tmp;

  return true;
}


} // namespace bm

