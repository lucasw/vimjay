#include "filter.h"

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

} // namespace bm

