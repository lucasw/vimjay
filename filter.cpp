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

}

bool Sobel::update()
{
  if (!ImageNode::update()) return false;
 
  if (out.empty()) {
    LOG(ERROR) << name << " out is empty";
    return false;
  }

  // TBD check for mismatches later
  if (tmp.empty()) { 
    tmp = out.clone(); 
  }

  cv::Sobel(out, tmp, out.depth(), 1, 1, 3, 8);

  out = tmp;

  return true;
}


} // namespace bm

