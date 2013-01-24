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
//////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////
  Webcam::Webcam(const std::string name) : 
      ImageNode(name),
      error_count(0)
  {
    setSignal("mode", 0, false, ROLL, 0, 4);

    LOG(INFO) << "camera opening ...";
    capture = VideoCapture(0); //CV_CAP_OPENNI );
    LOG(INFO) << "done.";

    if ( !capture.isOpened() ) {
      LOG(ERROR) << "Can not open a capture object.";
      return;// -1;
    }

        //bool rv1 = capture.set( CV_CAP_PROP_FRAME_WIDTH, 800);
        //    //bool rv2 = capture.set( CV_CAP_PROP_FRAME_HEIGHT, 600);
        //        //LOG(INFO) << "set res " << rv1 << " " << rv2;
        //
        //
    
    //update();
    //update();
    is_thread_dirty = false;
    cam_thread = boost::thread(&Webcam::runThread, this);

    // wait for single frame so there is a sample with the correct size
    // TBD or time out
    while(!is_thread_dirty && (error_count < 20)) {}
  }
  
  Webcam::~Webcam()
  {
    LOG(INFO) << name << " stopping camera capture thread";
    do_capture = false;
    run_thread = false;
    cam_thread.join();

    LOG(INFO) << name << " releasing the camera";
    capture.release();
  }

  void Webcam::runThread()
  {
    do_capture = true;
    run_thread = true;

    //cv::namedWindow("webcam", CV_GUI_NORMAL);


    while(run_thread) {
      if (do_capture && error_count < 20) {
        /// TBD need to make this in separate thread
        if( !capture.grab() )
        {
          LOG(ERROR) << name << " Can not grab images." << endl;
          error_count++;
          continue;
          //return false;
        } 

        cv::Mat new_out;
        capture.retrieve(new_out);

        if (new_out.empty()) {
          LOG(ERROR) << name << " new image empty";
          error_count++;
          continue;
          //return false;
        }

        error_count--;
        if (error_count < 0) error_count = 0;
        // I think opencv is reusing a mat within capture so have to clone it

        //if (&new_out.data == &out.data) {
        //
        //cv::Mat tmp; // new_out.clone();
       
        float scale = 1.0;
        if (MAT_FORMAT == CV_16S) scale = 255;
        if (MAT_FORMAT == CV_32F) scale = 1.0/255.0;
        if (MAT_FORMAT == CV_8U) scale = 1.0;
        cv::Mat tmp0 = cv::Mat(new_out.size(), CV_8UC4, cv::Scalar(0)); 
        // just calling reshape(4) doesn't do the channel reassignment like this does
        int ch[] = {0,0, 1,1, 2,2}; 
        mixChannels(&new_out, 1, &tmp0, 1, ch, 3 );
        cv::Mat tmp;
        tmp0.convertTo(tmp, MAT_FORMAT,scale); //, 1.0/(255.0));//*255.0*255.0*255.0));

        cv::Size sz = Config::inst()->getImSize();
        cv::Mat tmp1;
        cv::resize(tmp, tmp1, sz, 0, 0, getModeType() );
        

        //out_lock.lock();
        setImage("out", tmp1);
        is_thread_dirty = true;
        //out_lock.unlock();
        //} else {
        //  VLOG(3) << name << " dissimilar capture";
        //  out = new_out;
        //}

        // TBD out is the same address every time, why doesn't clone produce a new one?
        //VLOG(3) << 

      } else {
        usleep(1000);
      }
    }
  } // runThread

/*
  cv::Mat get()
  {

  }
  */

  bool Webcam::update()
  {
    // don't call ImageNode update because it will clobber the "out" image set in the thread
    Node::update();
    //ImageNode::update();

      if ( is_thread_dirty ) setDirty();
      is_thread_dirty = false;

    return true;
  }


///////////////////////////////////////////////////////////

  ImageDir::ImageDir(const std::string name) : Buffer(name) 
  {
    setSignal("mode", 0, false, ROLL, 0, 4);
    setSignal("keep_aspect", 1, false, ROLL, 0, 1);
    setString("dir", "../data"); //"temp");
    setString("name","");
    //setSignal("ind", 0, false, ROLL, 0, 0);
  }

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
   
    const bool keep_aspect = getSignal("keep_aspect");

    for (int i = 0; i < frames_orig.size(); i++) {
      cv::Mat tmp0 = frames_orig[i]; 
      cv::Size sz = Config::inst()->getImSize();
      
      cv::Mat tmp1 = cv::Mat( sz, tmp0.type(), cv::Scalar::all(0));
      
      const float aspect_0 = (float)tmp0.cols/(float)tmp0.rows;
      const float aspect_1 = (float)tmp1.cols/(float)tmp1.rows;
     
      if (keep_aspect) {

        cv::Size tmp_sz = sz;
        
        int off_x = 0;
        int off_y = 0;
        // TBD could have epsilon defined by 1 pixel width
        if (aspect_0 > aspect_1) {
          tmp_sz.height = tmp_sz.width / aspect_0;
          off_y = (sz.height - tmp_sz.height)/2;
        } else if (aspect_0 < aspect_1) {
          tmp_sz.width = tmp_sz.height * aspect_0;  
          off_x = (sz.width - tmp_sz.width)/2;
        }
        
        cv::Mat tmp_aspect;
        cv::resize( tmp0, tmp_aspect, tmp_sz, 0, 0, mode );
        
        // TBD put offset so image is centered
        cv::Mat tmp1_roi = tmp1(cv::Rect(off_x, off_y, tmp_sz.width, tmp_sz.height));
        tmp_aspect.copyTo(tmp1_roi);

        VLOG(3) << aspect_0 << " " << aspect_1 << ", " 
            << off_x << " " << off_y << " " << tmp_sz.width << " " << tmp_sz.height;
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

    if ((getSignal("load") > 0.5) || is_dirty1) {
      loadImages();
      resizeImages();
    } else {

      getSignal("mode", is_valid, is_dirty1, 72);
      getSignal("keep", is_valid, is_dirty2, 72);
     
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
    setString("cur_dir","", true);
    setSignal("ind", 0, false, ROLL, 0, 0);
    //cv::Mat tmp;
    //setImage("out", tmp);
  }

  bool BrowseDir::update()
  {
    const bool rv = Node::update();
    if (!rv) return false;
    
    if (!isDirty(this, 27)) return true;

    std::vector<string> image_names;
    std::vector<string> sub_dirs;
    
    const string dir = getString("dir");
    const bool rv2 = getImageNamesAndSubDirs( dir, image_names, sub_dirs );

    std::vector<int> num_sub_images;
    std::vector<int> num_sub_dirs;

    for (int i = 0; i < sub_dirs.size(); i++) 
    {
      std::vector<string> image_names2;
      std::vector<string> sub_dirs2;
      //const bool rv3 = getImageNamesAndSubDirs( dir + "/" + sub_dirs[i], image_names2, sub_dirs2);
      const bool rv3 = getImageNamesAndSubDirs( sub_dirs[i], image_names2, sub_dirs2);
      
      num_sub_dirs.push_back(sub_dirs2.size());
      num_sub_images.push_back(image_names2.size());
    }
    
    setSignal("ind", getSignal("ind"), false, ROLL, 0, sub_dirs.size()-1);
    
    const int ind = getSignal("ind");
    const string cur_dir = sub_dirs[ind];
    VLOG(1) << name << " " << cur_dir;
    setString("cur_dir", cur_dir);
    
    {
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

      for (int i = 0; i < image_names.size(); i++) {
        stringstream dir_info;

        dir_info << image_names[i];
       
        cv::Scalar col = cv::Scalar(200, 255, 170);
      
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
    if (key == '[') {
      setString("dir", getString("cur_dir"));
    } else {
      valid_key = false;
    }

    // TBD 
    //if (valid_key) setDirty();

    return valid_key;
  }

} //bm

