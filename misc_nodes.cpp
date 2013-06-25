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

#include "misc_nodes.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>

#include <deque>

#include "config.h"

using namespace cv;
using namespace std;

namespace bm {
///////////////////////////////////////////////////////////

  ImageDir::ImageDir(const std::string name) : Buffer(name) 
  {
    setSignal("mode", 0, false, ROLL, 0, 4);
    setSignal("keep_aspect", 1, false, ROLL, 0, 2);
    setString("dir", "../data"); //"temp");
    setString("name","");
    //setSignal("ind", 0, false, ROLL, 0, 0);
  }

  /**
    load a directory full of images

    TBD do in separate thread
    */
  bool ImageDir::loadImages()
  {
    // TBD this needs to go in separate thread

    boost::timer t1;
    std::string dir = getString("dir");
    LOG(INFO) << name << " loading " << dir;
   
    // TBD use getImageNamesAndSubdirs from utility.cpp
    boost::filesystem::path image_path(dir);
    if (!is_directory(image_path)) {
      LOG(ERROR) << name << CLERR << " not a directory " << CLNRM << dir; 
      return false;
    }

    // TBD clear frames first?
    vector<string> files;
    
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    for (boost::filesystem::directory_iterator itr( image_path );
        itr != end_itr;
        ++itr )
    {
      if ( is_directory( *itr ) ) continue;
      
      stringstream ss;
      ss << *itr;
      string next_im = ( ss.str() );
      // strip off "" at beginning/end
      next_im = next_im.substr(1, next_im.size()-2);
      files.push_back(next_im);
   }

   frames_orig.clear();

   sort(files.begin(), files.end());
  
   all_files.clear();
   for (int i=0; i < files.size(); i++) {
      const string next_im = files[i];
      cv::Mat new_out = cv::imread( next_im );
   
      if (new_out.data == NULL) { //.empty()) {
        LOG(WARNING) << name << " not an image? " << next_im;
        continue;
      }
   
     
      cv::Mat tmp0 = cv::Mat(new_out.size(), CV_8UC4, cv::Scalar(0)); 
        // just calling reshape(4) doesn't do the channel reassignment like this does
        int ch[] = {0,0, 1,1, 2,2}; 
        mixChannels(&new_out, 1, &tmp0, 1, ch, 3 );

        
      VLOG(1) << name << " " << i << " loaded image " << next_im;

      frames_orig.push_back(tmp0);
      all_files.push_back(next_im);
    }
    
    /// TBD or has sized increased since beginning of function?
    if (frames_orig.size() == 0) {
      LOG(ERROR) << name << CLERR << " no images loaded" << CLNRM << " " << dir;
      return false;
    }
    
    LOG(INFO) << name << " " << frames_orig.size() << " image loaded " << t1.elapsed();
    setSignal("ind", getSignal("ind"), false, ROLL, 0, frames_orig.size()-1);
    //max_size = frames.size() + 1;
    setDirty();

    return true;
  } // loadImages

  bool ImageDir::resizeImages()
  {
    const int mode = getModeType();
    //LOG(INFO) << "resize mode " << mode;

    boost::mutex::scoped_lock l(frames_mutex);
    frames.clear();
   
    const int keep_aspect = getSignal("keep_aspect");

    for (int i = 0; i < frames_orig.size(); i++) {
      cv::Mat tmp0 = frames_orig[i]; 
      cv::Size sz = Config::inst()->getImSize();
      cv::Mat tmp1 = cv::Mat( sz, tmp0.type(), cv::Scalar::all(0));
     
      if (keep_aspect == 1) {
        fixAspect(tmp0, tmp1, mode);
      } else if (keep_aspect == 2) {
        fixAspectFill(tmp0, tmp1, mode);
      } else {
        cv::resize( tmp0, tmp1, sz, 0, 0, mode );
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
  }

  bool ImageDir::save(cv::FileStorage& fs) 
  {
    Buffer::save(fs);

    fs << "dir" << getString("dir");
  }

  bool ImageDir::update()
  {
    const bool rv = Buffer::update();
    if (!rv) return false;
    
    if (!isDirty(this, 27)) return true;

    bool is_valid, is_dirty1, is_dirty2;
    getString("dir", is_valid, is_dirty1, 72);

    if ((getBool("load")) || is_dirty1) {
      loadImages();
      resizeImages();
    } else {

      getSignal("mode", is_valid, is_dirty1, 72);
      getSignal("keep_aspect", is_valid, is_dirty2, 72);
     
      if (is_dirty1 || is_dirty2) {
        resizeImages();
      }
      
    }

    // flush dirtiness, TBD is this necessary
    //isDirty(this, 27);

    return true;
  }

 
  ///////////////////////////////////////////////////////////////
  BrowseDir::BrowseDir(const std::string name) : ImageNode(name) 
  {
    setString("dir", "../data"); //"temp");
    setString("old_dir", "../data"); //"temp");
    setString("cur_dir","", true);
    setSignal("ind", 0, false, ROLL, 0, 0);
    //cv::Mat tmp;
    //setImage("out", tmp);
    
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
    while (true) {
      bool dir_valid, dir_dirty;
      const string dir = getString("dir", dir_valid, dir_dirty);

      if (dir_valid && dir_dirty) {
        setSignal("is_updating", 1, true);
       
        std::vector<string> file_names_tmp;
        // TBD make a struct for these
        std::vector<string> sub_dirs_tmp;
        std::vector<int> num_sub_images_tmp;
        std::vector<int> num_sub_dirs_tmp;

        const bool rv2 = getImageNamesAndSubDirs( dir, file_names_tmp, sub_dirs_tmp );
        if (rv2) {

        // filter out the unusable/bad directories
        for (int i = 0; i < sub_dirs_tmp.size(); i++) 
        {
          std::vector<string> file_names2;
          std::vector<string> sub_dirs2;
          //const bool rv3 = getImageNamesAndSubDirs( dir + "/" + sub_dirs[i], file_names2, sub_dirs2);
          const bool rv3 = getImageNamesAndSubDirs( 
              sub_dirs_tmp[i], file_names2, sub_dirs2);
          //if (!rv3) continue;

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

        } else {
          // bad dir
          LOG(INFO) << "restoring old dir " << getString("old_dir");
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
      
    setSignal("ind", getSignal("ind"), false, ROLL, 0, 
        file_names.size() + sub_dirs.size() - 1);
   
    const int ind = getSignal("ind");
    const int file_ind = ind - sub_dirs.size();
    bool highlight_dir_not_file = true;

    if ((sub_dirs.size() > 0) && (ind < sub_dirs.size())) {
      const string cur_dir = sub_dirs[ind];
      VLOG(1) << name << " dir " << cur_dir;
      setString("cur_dir", cur_dir);
      highlight_dir_not_file = true;
    } else if ((file_names.size() > 0) && (file_ind >= 0) && 
        (file_ind < file_names.size())) {
      const string cur_file = file_names[file_ind];
      VLOG(1) << name << " file " << cur_file;
      setString("cur_file", cur_file);
      highlight_dir_not_file = false;
    }

    {
      // the assumption is none of this takes very long, so no point
      // in making copies of the dir vectors
      boost::mutex::scoped_lock l(dirs_mutex);
      cv::Mat out = cv::Mat( Config::inst()->getImSize(), CV_8UC4, cv::Scalar::all(0) );
      
      // make text scroll with selection
      int offset = 0;
      const int text_rows = out.rows/10;
      if (ind > text_rows/2) offset -= (ind - text_rows/2)*10;

      for (int i = 0; i < sub_dirs.size(); i++) {
        stringstream dir_info;

        dir_info << sub_dirs[i] << " " << num_sub_images[i] << " " << num_sub_dirs[i];
       
        cv::Scalar col = cv::Scalar(200, 200, 200);

        if (i == ind) {
          dir_info << " " << ind;
          col = cv::Scalar(200, 200, 255);
        }
      
        cv::putText(out, dir_info.str(), cv::Point(10, 10 + 10*i + offset), 1, 1, col, 1);
      }

      offset += sub_dirs.size()*10;

      for (int i = 0; i < file_names.size(); i++) {
        stringstream dir_info;

        dir_info << file_names[i];
       
        cv::Scalar col = cv::Scalar(200, 255, 170);
 
        if (i == file_ind) {
          dir_info << " " << file_ind;
          col = cv::Scalar(200, 200, 255);
        }
     
        cv::putText(out, dir_info.str(), cv::Point(10, 10 + 10*i + offset), 1, 1, col, 1);
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
    if (key == 13) { // enter key
      // descend into currently selected directory, save the old one
      setString("old_dir", getString("dir"));
      setString("dir", getString("cur_dir"));
    } else if (key == '\'') { 
      // go up to parent directory

      const std::string cur_dir = getString("dir");
     
      const int len = cur_dir.size();
      std::string parent_dir;

      if ((len >= 2) && (cur_dir[len-2] == '.') && (cur_dir[len-1] == '.')) {
        parent_dir = cur_dir + "/..";
      } else {
        const size_t pos = cur_dir.find_last_of("/");
        parent_dir = cur_dir.substr(0, pos);
        if (parent_dir == cur_dir) {
          parent_dir = parent_dir + "/..";
        }
      }
      
      try {
      const boost::filesystem::path image_path(parent_dir);
      // validate before setting
      if (is_directory(image_path)) {
        LOG(INFO) << "parent dir " << cur_dir << " -> " << parent_dir;
        // or provide way to reset value to default
        setString("old_dir", getString("dir"));
        setString("dir", parent_dir);
      } else {
        LOG(INFO) << "parent dir is not valid (could be at root), not using " 
            << parent_dir;
      }
      } catch (boost::filesystem3::filesystem_error& ex) {
        LOG(ERROR) << "bad dir " << parent_dir << " "
            << boost::diagnostic_information(ex);
      }

    } else {
      valid_key = false;
    }

    // TBD 
    //if (valid_key) setDirty();

    return valid_key;
  }

} //bm

