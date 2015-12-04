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



#include "config.h"

#include <iostream>
#include <ros/console.h>

#include "opencv2/highgui/highgui.hpp"

// DEFINE_string(config_file, "../config.yml", "configuration settings for ui and output");

namespace bm
{


Config* Config::instance = NULL;

Config* Config::inst()
{
  if (!instance)
  {
    instance = new Config;

    instance->ui_width = 1280;
    instance->ui_height = 720;
    instance->thumb_width = 40;
    instance->thumb_height = 30;
    instance->im_width = 640;
    instance->im_height = 480;

    // TBD
    // instance->load(FLAGS_config_file);
  }

  return instance;
}

bool Config::load(const std::string config_file)
{
  ROS_INFO_STREAM("loading config " << config_file);

  cv::FileStorage fs;
  fs.open(config_file, cv::FileStorage::READ);

  if (!fs.isOpened())
  {
    ROS_ERROR_STREAM("couldn't open " << config_file);
    return false;
  }

  fs["ui_width"] >> ui_width;
  fs["ui_height"] >> ui_height;
  fs["thumb_width"] >> thumb_width;
  fs["thumb_height"] >> thumb_height;
  fs["im_width"] >> im_width;
  fs["im_height"] >> im_height;
  fs["out_width"] >> out_width;
  fs["out_height"] >> out_height;

  ROS_INFO_STREAM("UI " << ui_width << " " << ui_height);
  ROS_INFO_STREAM("thumb " << thumb_width << " " << thumb_height);
  ROS_INFO_STREAM("output " << im_width << " " << im_height);

  // key assignment, TBD put into yaml
  // also check for duplicates?  Make node name specific?
  key_map["VAL_DOWN"] = '.';
  key_map["VAL_UP"] = '/';
  key_map["VAL_UP2"] = ',';
  key_map["VAL_DOWN2"] = 'm';
  key_map["VAL_ZERO"] = 'n';
}


};
