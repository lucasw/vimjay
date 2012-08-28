
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

};
#endif
