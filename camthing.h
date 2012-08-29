
#ifndef __CAMTHING_H__
#define __CAMTHING_H__

#include <string.h>

#include "nodes.h"

namespace bm {

  class Config
  {
    private:
      Config() {};
      Config(Config const&) {};
      Config& operator=(Config const&) {};

      static Config* instance;
    public:
      static Config* inst();

      int width;
      int height;
      int thumb_width;
      int thumb_height;
      int im_width;
      int im_height;

      bool load(const std::string file);
  };

  std::string getId(Node* ptr);


std::string logMat(const cv::Mat& m);
bool getBezier(
    const std::vector<cv::Point2f>& control_points, // TBD currently has to be 4
    std::vector<cv::Point2f>& output_points,
    const int num // numbe of intermediate points to generate 
    );

};
#endif
