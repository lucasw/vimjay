/*
  
  Copyright 2012 Lucas Walter

     This file is part of Camthing.

    Camthing is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Camthing is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Camthing.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "misc_nodes.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>

#include <deque>

using namespace cv;
using namespace std;

namespace bm {
//////////////////////////////////////////////////
  Saw::Saw() : Signal()
  {
    vcol = cv::Scalar(0,90,255);
  }
  
  void Saw::setup(const float new_step, const float offset, const float min, const float max) 
  {
    Signal::setup(new_step, offset, min, max);
  }

  bool Saw::handleKey(int key)
  {
    bool valid_key = Signal::handleKey(key);
    if (valid_key) return true;
    
    //valid_key = true;

    // TBD 
    if (valid_key) setDirty();
    
    return valid_key;
  }

  bool Saw::update()
  {
    // don't call Signal::update because it will contradict this update
    if (!Node::update()) return false;

    float step = getSignal("step");
    float min = getSignal("min");
    float max = getSignal("max");
    float value = getSignal("value");

    value += step;
    if (value > max) {
      step = -abs(step);
      value = max;
    }
    if (value < min) {
      step = abs(step);
      value = min;
    }

    setSignal("step", step);
    setSignal("value", value);

    setDirty();
    //is_dirty = true;

    VLOG(3) << "Signal " << name << " " << value;
    return true;
  }

  //
  Rot2D::Rot2D()
  {

    setSignal("angle", 0);
    setSignal("scale", 1.0);
    setSignal("center_x", 0.0);
    setSignal("center_y", 0.0);
    cv::Mat tmp;
    setImage("in",tmp);
  }

  bool Rot2D::update()
  {
    if (!Node::update()) return false;

    // anything to rotate?
    if (inputs.size() < 1) return false;
   
    bool im_dirty;
    cv::Mat tmp_in;
    // "image" is the default image input name
    tmp_in = getImage("in");
    if (tmp_in.empty()) return false;

    float angle = getSignal("angle");    
    float scale = getSignal("scale");    

    cv::Point2f center;
    center.x = getSignal("center_x");     
    center.y = getSignal("center_y");

    //VLOG(1) << name << " " << is_dirty << " " << im_in->name << " " << im_in->is_dirty;

    cv::Mat rot = cv::getRotationMatrix2D(center, angle, scale);
    cv::Mat tmp;
    cv::warpAffine(tmp_in, tmp, rot, tmp_in.size(), INTER_NEAREST);

    setImage("out", tmp);
  }

  ////////////////////////////////////////////////////////////
  Webcam::Webcam() : error_count(0)
  {
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
    cam_thread = boost::thread(&Webcam::runThread,this);

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
        cv::Mat tmp;
        new_out.convertTo(tmp, MAT_FORMAT,scale); //, 1.0/(255.0));//*255.0*255.0*255.0));

        //out_lock.lock();
        setImage("out", tmp);
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
    Node::update();
    //ImageNode::update();

      if ( is_thread_dirty ) setDirty();
      is_thread_dirty = false;

    return true;
  }


///////////////////////////////////////////////////////////
  bool ImageDir::loadImages()
  {
    LOG(INFO) << name << " loading " << dir;

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

   sort(files.begin(), files.end());
  
   for (int i=0; i < files.size(); i++) {
      const string next_im = files[i];
      cv::Mat tmp0 = cv::imread( next_im );
     
      if (tmp0.data == NULL) { //.empty()) {
        LOG(WARNING) << name << " not an image? " << next_im;
        continue;
      }
      
      LOG(INFO) << name << " " << i << " loaded image " << next_im;

      cv::Size sz = cv::Size(640,480);
      cv::Mat tmp1;
      cv::resize(tmp0, tmp1, sz, 0, 0, cv::INTER_NEAREST );

      const bool restrict_size = false;
      const bool rv = add(tmp1, restrict_size);
    }
    
    /// TBD or has sized increased since beginning of function?
    if (frames.size() == 0) {
      LOG(ERROR) << name << CLERR << " no images loaded" << CLNRM;
      return false;
    }
    
    LOG(INFO) << name << " " << frames.size() << " image loaded";
    //max_size = frames.size() + 1;
    
    return true;
  }

  bool ImageDir::load(cv::FileNodeIterator nd)
  {
    Buffer::load(nd);
    
    (*nd)["dir"] >> dir;

    loadImages();
  }

  bool ImageDir::save(cv::FileStorage& fs) 
  {
    Buffer::save(fs);

    fs << "dir" << dir;
  }

///////////////////////////////////////////////////////////
  Tap::Tap() : ImageNode()
  {
    vcol = cv::Scalar(100, 30, 250);
  }

  void Tap::setup(Signal* new_signal, Buffer* new_buffer) 
  {
    inputs.clear();
    // TBD need caller to provide these
    setInputPort("Signal","value", new_signal, "value");
    setInputPort("Buffer","buffer", new_buffer, "out");
  }

  bool Tap::update()
  {
    if (!Node::update()) return false;

    if (isDirty(this,4)) {
      float value = getSignal("value");     
      
      VLOG(1) << name << " update " << value;
      cv::Mat tmp;
      if (!getBuffer("buffer", value, tmp)) return false;
      
      setImage("out", tmp);
    }

    return true;
  }

  bool Tap::draw() 
  {
    ImageNode::draw();

    stringstream sstr;
    sstr << getSignal("value");
    cv::putText(graph, sstr.str(), loc + cv::Point(20,-30), 1, 1, cv::Scalar(200,200,200));
  }
  
  bool TapInd::update()
  {
    if (!Node::update()) return false;

    if (isDirty(this, 4)) {
      float value =  getSignal("value");     
      int ind = value;

      VLOG(2) << name << " update " << ind;
      cv::Mat tmp;
      if (!getBuffer("buffer", ind, tmp)) return false;
      
      setImage("out", tmp);
    }

    return true;
  }
  
  bool TapInd::draw() 
  {
    Tap::draw();

    stringstream sstr;
    sstr << (int)(getSignal("value"));
    cv::putText(graph, sstr.str(), loc - cv::Point(-20,-30), 1, 1, cv::Scalar(200,200,200));
  
    return true;
  }

  ///////////////////////////////////////////
  Add::Add() : ImageNode()
  {
    cv::Mat tmp;
    setImage("add0", tmp);
    setSignal("add0", 2.0);
    vcol = cv::Scalar(200, 200, 50);
  }
  
  void Add::setup(vector<ImageNode*> np, vector<float> nf) 
  {
    if (np.size() != nf.size()) {
      LOG(ERROR) << CLWRN << "mismatched inputs and coefficients" << CLNRM;
      //return; 
    }
   
    // TBD instead of clearing all, only clear the keys that match "add"
    inputs["ImageNode"].clear();
    inputs["Signal"].clear(); 
    for (int i = 0; i < np.size(); i++) {
      const string port = "add" + boost::lexical_cast<string>(i);
      setInputPort("ImageNode", port, np[i], "out");
      setInputPort("Signal", port, NULL, "value"); // this allows other signals to connect to replace nf
    }
  }

  bool Add::update()
  {
    if (!Node::update()) return false;

    //VLOG(1) << "name " << is_dirty << " " << p1->name << " " << p1->is_dirty << ", " << p2->name << " " << p2->is_dirty ;
    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      // TBD accomodate bad mats somewhere

      cv::Size sz;

      bool done_something = false;
      
      cv::Mat out;

      // TBD should these vectors just be stored with some incrementing string?
      // TBD loop through all input ImageNodes and input signals (or vector values, need to be able
      // to handle either)
      // TBD instead of having nf, loop through all svals and match on ones that start with add
      // and then run getSignal on those strings
      for (map<string, float >::iterator it = svals.begin(); it != svals.end(); it++) {
      //for (int i = 0; i < nf.size(); i++) {
        const string port = it->first;
        
        if (port.substr(0,3) != "add") {
          VLOG(1) << name << " : " << port.substr(0,3) << " " << port;
          continue;
        }
        
        cv::Mat tmp_in;
        bool im_dirty;
        //const string port = "add" + boost::lexical_cast<string>(i);
        tmp_in = getImage(port);
        if (tmp_in.empty()) {
          VLOG(1) << name << " : " << port << " image is empty"; 
          continue;
        }

        float val = getSignal(port);
        
        // with 8-bit unsigned it is necessary to have initial coefficients positive
        // zero minus anything will just be zero 
        if (!done_something) {
          out = tmp_in * val;
          sz = tmp_in.size();
          done_something = true;
        } else { 

          if (sz != tmp_in.size()) {
            LOG(ERROR) << name << " size mismatch " << sz.width << " " << sz.height 
                << " != " << tmp_in.size().width << " " << tmp_in.size().height ;
            continue;
          }
         
          if (val > 0)
            out += tmp_in * val;
          else 
            out -= tmp_in * -val;

        }
      } // nf loop
      
    setImage("out", out);

    return true;
  }

  bool Add::load(cv::FileNodeIterator nd)
  {
    ImageNode::load(nd);
    /*
    FileNode nd2 = nd["nf"];
    if (nd2.type() != FileNode::SEQ) {
      LOG(ERROR) << "no nodes";
      return false;
    }
    */

    /*
    for (int i = 0; i < (*nd)["nf"].size(); i++) {
      float new_coeff;
      (*nd)["nf"][i] >> new_coeff;
      nf.push_back(new_coeff);
    }
    */
  }

  bool Add::save(cv::FileStorage& fs)
  {
    ImageNode::save(fs);

    /*
    fs << "nf" << "[:";
    for (int i = 0; i < nf.size(); i++) { 
      fs << nf[i]; 
    }
    fs << "]";
    */
  }

  ////////////////////////////////////////
  Resize::Resize() 
  {
    setSignal("fx", 0.2);
    setSignal("fy", 0.2);
    cv::Mat tmp;
    setImage("in", tmp);
  }

  bool Resize::update()
  {
  if (!ImageNode::update()) return false;
 
  cv::Mat in = getImage("in");

  if (in.empty()) {
    VLOG(2) << name << " in is empty";
    return false;
  }
  
  // if fx and fy aren't hooked up then they will remain unaltered

  cv::Size sz = in.size();
  
  float fx = abs(getSignal("fx"));
  float fy = abs(getSignal("fy"));
  
  cv::Size dsize = cv::Size(fx*sz.width, fy*sz.height);
  //TBD
  if (fx > 1.0) dsize.width = sz.width/fx;
  if (fy > 1.0) dsize.height = sz.height/fy;

  if (dsize.width < 1) dsize.width = 1;
  if (dsize.height < 1) dsize.height = 1;
  // TBD
  if (dsize.height > sz.height) dsize.height= sz.height;
  if (dsize.width > sz.width) dsize.width = sz.width;

  cv::Mat tmp;
  cv::resize(in, tmp, dsize, 0, 0, cv::INTER_NEAREST);
  // then scale back to input size
  cv::Mat out;

  cv::resize(tmp, out, sz, 0, 0, cv::INTER_NEAREST);
  setImage("out", out);
  VLOG(1) << fx << " " << fy << " " 
      << tmp.size().width << " " << tmp.size().height
      << " " << out.size().width << " " << out.size().height;
  return true;
  }

  bool Resize::draw() 
  {
    ImageNode::draw();

    stringstream sstr;
    sstr << getSignal("fx") << " " << getSignal("fy");
    cv::putText(graph, sstr.str(), loc + cv::Point(20,-40), 1, 1, cv::Scalar(200,200,200));
    //VLOG(1)<< sstr.str();
    return true;
  }

} //bm

