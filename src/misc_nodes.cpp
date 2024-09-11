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

#include "vimjay/misc_nodes.h"

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

///////////////////////////////////////////////////////////////
BrowseDir::BrowseDir(const std::string name) : ImageNode(name)
{
}

void BrowseDir::init()
{
  ImageNode::init();
  setString("dir", "../data");  // "temp");
  setString("old_dir", "../data");  // "temp");
  setString("selection", "", true);
  setSignal("ind", 0, false, ROLL, 0, 0);
  // cv::Mat tmp;
  // setImage("out", tmp);

  browse_thread = boost::thread(&BrowseDir::runThread, this);
}

void BrowseDir::runThread()
{

  /*
     The idea is that this runs only when "dir" is changed,
     if it takes a long time to finish then it will be dirty
     again when it starts over (if it changed multiple times
     it will only get the very last value
    */
  while (true)
  {
    bool dir_valid, dir_dirty;
    const string dir = getString("dir", dir_valid, dir_dirty);

    if (dir_valid && dir_dirty)
    {
      setSignal("is_updating", 1, true);

      std::vector<string> file_names_tmp;
      // TBD make a struct for these
      std::vector<string> sub_dirs_tmp;
      std::vector<int> num_sub_images_tmp;
      std::vector<int> num_sub_dirs_tmp;

      const bool rv2 = getImageNamesAndSubDirs(dir, file_names_tmp, sub_dirs_tmp);
      if (rv2)
      {

        // filter out the unusable/bad directories
        for (int i = 0; i < sub_dirs_tmp.size(); i++)
        {
          std::vector<string> file_names2;
          std::vector<string> sub_dirs2;
          // const bool rv3 = getImageNamesAndSubDirs( dir + "/" + sub_dirs[i], file_names2, sub_dirs2);
          const bool rv3 = getImageNamesAndSubDirs(
                             sub_dirs_tmp[i], file_names2, sub_dirs2);
          // if (!rv3) continue;

          num_sub_dirs_tmp.push_back(sub_dirs2.size());
          num_sub_images_tmp.push_back(file_names2.size());
        }

        {
          // lock the class variables while updating
          boost::mutex::scoped_lock l(dirs_mutex);
          file_names = file_names_tmp;
          sub_dirs = sub_dirs_tmp;
          num_sub_images = num_sub_images_tmp;
          num_sub_dirs = num_sub_dirs_tmp;
        }

      }
      else
      {
        // bad dir
        ROS_INFO_STREAM("restoring old dir " << getString("old_dir"));
        setString("dir", getString("old_dir"));
      }

      setSignal("is_updating", 0);
      setDirty();
    }

    sleep(1);
  }
}


bool BrowseDir::update()
{
  const bool rv = Node::update();
  if (!rv) return false;

  if (!isDirty(this, 27)) return true;

  const int ind_size = file_names.size() + sub_dirs.size() - 1;
  setSignal("ind", getSignal("ind"), false, ROLL, 0, ind_size);

  const int ind = getSignal("ind");
  const int file_ind = ind - sub_dirs.size();
  bool highlight_dir_not_file = true;

  // TBD this is ugly
  // index into directories
  if ((sub_dirs.size() > 0) && (ind < sub_dirs.size()))
  {
    const string cur_dir = sub_dirs[ind];
    // ROS_DEBUG_STREAM_COND(log_level > 1, name << " dir " << cur_dir);
    setString("cur_sel", cur_dir);
    highlight_dir_not_file = true;
    // index into files
  }
  else if ((file_names.size() > 0) && (file_ind >= 0) &&
           (file_ind < file_names.size()))
  {
    const string cur_file = file_names[file_ind];
    // ROS_DEBUG_STREAM_COND(log_level > 1, name << " file " << cur_file);
    setString("cur_sel", cur_file);
    highlight_dir_not_file = false;
  }

  {
    // the assumption is none of this takes very long, so no point
    // in making copies of the dir vectors
    boost::mutex::scoped_lock l(dirs_mutex);
    cv::Mat out = cv::Mat(Config::inst()->getImSize(), CV_8UC4, cv::Scalar::all(0));

    // make text scroll with selection
    int offset = 0;
    const int text_rows = out.rows / 10;
    if (ind > text_rows / 2) offset -= (ind - text_rows / 2) * 10;

    for (int i = 0; i < sub_dirs.size(); i++)
    {
      stringstream dir_info;

      dir_info << sub_dirs[i] << " " << num_sub_images[i] << " " << num_sub_dirs[i];

      cv::Scalar col = cv::Scalar(200, 200, 200);

      if (i == ind)
      {
        dir_info << " " << ind;
        col = cv::Scalar(200, 200, 255);
      }

      cv::putText(out, dir_info.str(), cv::Point(10, 10 + 10 * i + offset), 1, 1, col, 1);
    }

    offset += sub_dirs.size() * 10;

    for (int i = 0; i < file_names.size(); i++)
    {
      stringstream dir_info;

      dir_info << file_names[i];

      cv::Scalar col = cv::Scalar(200, 255, 170);

      if (i == file_ind)
      {
        dir_info << " " << file_ind;
        col = cv::Scalar(200, 200, 255);
      }

      cv::putText(out, dir_info.str(), cv::Point(10, 10 + 10 * i + offset), 1, 1, col, 1);
    }

    setImage("out", out);

  }

  return true;
}


bool BrowseDir::handleKey(int key)
{
  bool valid_key = ImageNode::handleKey(key);
  if (valid_key) return true;

  valid_key = true;
  if (key == 39)   // '
  {
    // descend into currently selected directory, save the old one
    const string cur_sel = getString("cur_sel");
    const boost::filesystem::path image_path(cur_sel);
    // validate before setting
    if (is_directory(image_path))
    {
      setString("old_dir", getString("dir"));
      setString("dir", cur_sel);
    }
  }
  else if (key == 59)     // ;
  {
    // go up to parent directory

    const std::string cur_dir = getString("dir");

    const int len = cur_dir.size();
    std::string parent_dir;

    if ((len >= 2) && (cur_dir[len - 2] == '.') && (cur_dir[len - 1] == '.'))
    {
      parent_dir = cur_dir + "/..";
    }
    else
    {
      const size_t pos = cur_dir.find_last_of("/");
      parent_dir = cur_dir.substr(0, pos);
      if (parent_dir == cur_dir)
      {
        parent_dir = parent_dir + "/..";
      }
    }

    try
    {
      const boost::filesystem::path image_path(parent_dir);
      // validate before setting
      if (is_directory(image_path))
      {
        ROS_INFO_STREAM("parent dir " << cur_dir << " -> " << parent_dir);
        // or provide way to reset value to default
        setString("old_dir", getString("dir"));
        setString("dir", parent_dir);
      }
      else
      {
        ROS_INFO_STREAM("parent dir is not valid (could be at root), not using "
                        << parent_dir);
      }
    }
    catch (boost::filesystem::filesystem_error& ex)
    {
      //} catch (boost::filesystem3::filesystem_error& ex) {
      ROS_ERROR_STREAM("bad dir " << parent_dir << " "
                       << boost::diagnostic_information(ex));
    }
  }
  else if (key == 13)     // enter key
  {
    // confirm the selection
    setString("selection", getString("cur_sel"));
  }
  else
  {
    valid_key = false;
  }

  // TBD
  // if (valid_key) setDirty();

  return valid_key;
}

}  // namespace bm

