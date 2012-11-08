#ifndef __CONFIG_H__
#define __CONFIG_H__


#include <iostream>
#include <sstream>
#include <stdio.h>
#include <time.h>

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

      int ui_width;
      int ui_height;
      int thumb_width;
      int thumb_height;
      int im_width;
      int im_height;
      int out_width;
      int out_height;

      bool load(const std::string file);
  };

};

#endif // __CONFIG_H__
