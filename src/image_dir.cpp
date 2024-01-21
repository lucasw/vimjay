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

#include "vimjay/image_dir.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/timer/timer.hpp>

#include <ros/console.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <deque>

#include "vimjay/config.h"

using namespace cv;
using namespace std;

namespace bm
{
///////////////////////////////////////////////////////////

ImageDir::ImageDir(const std::string name) : Buffer(name)
{
}

void ImageDir::init()
{
  Buffer::init();
  setSignal("mode", 0, false, ROLL, 0, 4);
  setSignal("keep_aspect", 1, false, ROLL, 0, 2);

  std::string dir = "data";
  // ros::param::get
  Config::inst()->nh_.getParam("image_dir", dir);
  setString("dir", dir);
  setString("name", "");
  // setSignal("ind", 0, false, ROLL, 0, 0);
}

/**
  load a directory full of images

  TBD do in separate thread
  */
bool ImageDir::loadImages()
{
  // TBD this needs to go in separate thread

  boost::timer::cpu_timer t1;
  std::string dir = getString("dir");
  ROS_INFO_STREAM(name << " loading " << dir);

  // TBD use getImageNamesAndSubdirs from utility.cpp
  boost::filesystem::path image_path(dir);
  if (!is_directory(image_path))
  {
    ROS_ERROR_STREAM(name << CLWRN << " not a directory " << CLNRM << dir);
    return false;
  }

  // TBD clear frames first?
  vector<string> files;

  boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
  for (boost::filesystem::directory_iterator itr(image_path);
       itr != end_itr;
       ++itr)
  {
    if (is_directory(*itr)) continue;

    stringstream ss;
    ss << *itr;
    string next_im = (ss.str());
    // strip off "" at beginning/end
    next_im = next_im.substr(1, next_im.size() - 2);
    files.push_back(next_im);
  }

  frames_orig.clear();

  sort(files.begin(), files.end());

  all_files.clear();
  for (int i = 0; i < files.size(); i++)
  {
    const string next_im = files[i];
    cv::Mat new_out = cv::imread(next_im);

    if (new_out.data == NULL)   // .empty()) {
    {
      ROS_WARN_STREAM(name << " not an image? " << next_im);
      continue;
    }


    cv::Mat tmp0 = cv::Mat(new_out.size(), CV_8UC4, cv::Scalar(0));
    // just calling reshape(4) doesn't do the channel reassignment like this does
    int ch[] = {0, 0, 1, 1, 2, 2};
    mixChannels(&new_out, 1, &tmp0, 1, ch, 3);


    ROS_DEBUG_STREAM_COND(log_level > 1, name << " " << i << " loaded image " << next_im);

    frames_orig.push_back(tmp0);
    all_files.push_back(next_im);
  }

  /// TBD or has sized increased since beginning of function?
  if (frames_orig.size() == 0)
  {
    ROS_ERROR_STREAM(name << CLERR << " no images loaded" << CLNRM << " " << dir);
    return false;
  }

  ROS_INFO_STREAM(name << " " << frames_orig.size() << " image loaded " << t1.format());
  setSignal("ind", getSignal("ind"), false, ROLL, 0, frames_orig.size() - 1);
  // max_size = frames.size() + 1;
  setDirty();

  return true;
} // loadImages

bool ImageDir::resizeImages()
{
  const int mode = getModeType();
  // ROS_INFO_STREAM("resize mode " << mode);

  boost::mutex::scoped_lock l(frames_mutex);
  frames.clear();

  const int keep_aspect = getSignal("keep_aspect");

  for (int i = 0; i < frames_orig.size(); i++)
  {
    cv::Mat tmp0 = frames_orig[i];
    cv::Size sz = Config::inst()->getImSize();
    cv::Mat tmp1 = cv::Mat(sz, tmp0.type(), cv::Scalar::all(0));

    if (keep_aspect == 1)
    {
      fixAspect(tmp0, tmp1, mode);
    }
    else if (keep_aspect == 2)
    {
      fixAspectFill(tmp0, tmp1, mode);
    }
    else
    {
      cv::resize(tmp0, tmp1, sz, 0, 0, mode);
    }

    const bool restrict_size = false;

    const bool rv = addCore(tmp1, restrict_size);
  }

  setDirty();
  return true;
}

bool ImageDir::load(cv::FileNodeIterator nd)
{
  Buffer::load(nd);

  string dir;
  (*nd)["dir"] >> dir;

  setString("dir", dir);

  loadImages();
  resizeImages();

  return true;
}

bool ImageDir::save(cv::FileStorage& fs)
{
  Buffer::save(fs);

  fs << "dir" << getString("dir");

  return true;
}

bool ImageDir::update()
{
  const bool rv = Buffer::update();
  if (!rv) return false;

  if (!isDirty(this, 27)) return true;

  bool is_valid, is_dirty1, is_dirty2;
  getString("dir", is_valid, is_dirty1, 72);

  if ((getBool("load")) || is_dirty1)
  {
    loadImages();
    resizeImages();
  }
  else
  {

    getSignal("mode", is_valid, is_dirty1, 72);
    getSignal("keep_aspect", is_valid, is_dirty2, 72);

    if (is_dirty1 || is_dirty2)
    {
      resizeImages();
    }

  }

  // flush dirtiness, TBD is this necessary
  // isDirty(this, 27);

  return true;
}
}  // namespace bm

