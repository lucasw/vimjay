#ifndef __CONFIG_H__
#define __CONFIG_H__


#include <map>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <time.h>

#include "opencv2/highgui/highgui.hpp"

namespace bm {

  class Config
  {
    private:
      Config() {};
      Config(Config const&) {};
      Config& operator=(Config const&) {};

      static Config* instance;

    // assignment of keys to function
    std::map<const std::string, int> key_map;
      
    public:
      static Config* inst();

      int key(const std::string key_key) {
        return key_map[key_key];
      }

      int ui_width;
      int ui_height;
      int thumb_width;
      int thumb_height;
      int out_width;
      int out_height;

      bool load(const std::string file);

      int im_width;
      int im_height;
      cv::Size getImSize() { return cv::Size(im_width, im_height); }

      // TBD register every Elem creation from the constructor
      // and destructor- debug if clearNodes is doing right thing,
      // or specific node deletion 
  };

};

#endif // __CONFIG_H__
