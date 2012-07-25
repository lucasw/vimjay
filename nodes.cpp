
#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>

#include <deque>
//#include <pair>

#include "nodes.h"

using namespace cv;
using namespace std;

namespace bm {


  ////////////////////////////////////////////////
  Node::Node() {
    do_update = false;
    is_dirty = true;
    vcol = cv::Scalar(0,128,255);
  }
  
  bool Node::setUpdate()
  {
    if (do_update) return false;

    do_update = true;

    //LOG(INFO) << "in sz " << inputs.size();
    for (int i = 0; i < inputs.size(); i++) {
      inputs[i]->setUpdate();  
    }
    
    return true;
  }

  // the rv is so that an ineritanning function will know whether to 
  // process or not
  bool Node::update() 
  {
    if (!do_update) return false;
    do_update = false; 

    bool inputs_dirty = is_dirty;
    for (int i = 0; i < inputs.size(); i++) {
      inputs[i]->update();
      if (inputs[i]->is_dirty) inputs_dirty = true;
    }

    // the inheriting object needs to set is_dirty as appropriate?
    is_dirty = inputs_dirty;
    
    VLOG(2) << name << " in sz " << inputs.size() << " " << is_dirty;

    return true;
  }

  bool Node::draw(float scale) 
  {
    int fr = 1;
    if (!is_dirty) fr = 5;
    
    cv::Scalar col = cv::Scalar(vcol/fr);

    cv::circle(graph, loc, 20, col, 4);
  
    for (int j = 0; j < inputs.size(); j++) {
      cv::Point src = inputs[j]->loc;
      cv::Point mid = src + (loc -src) * 0.8;
      cv::line( graph, src, mid, cv::Scalar(0, 128/fr, 0), 2, 4 );
      cv::line( graph, mid, loc, cv::Scalar(0, 255/fr, 0), 2, CV_AA );
    }

  }

  //////////////////////////////////
  ImageNode::ImageNode() : Node()
  {
    vcol = cv::Scalar(255,0,255);
  }

  // TBD could there be a templated get function to be used in different node types?
  cv::Mat ImageNode::get() {
    return out;//_old;
  }
  
  bool ImageNode::update() 
  {
    const bool rv = Node::update();
    
    if (!rv) return false;

    if (inputs.size() > 0) {
      ImageNode* im_in = dynamic_cast<ImageNode*> (inputs[0]);
      if (im_in) {
        
        //out_old = out;//.clone(); // TBD need to clone this?  It doesn't work
        cv::Mat new_out = im_in->get(); 

        out_old = out;
        if (new_out.refcount == out.refcount) {
          VLOG(2) << "dirty input is identical with old image " << new_out.refcount << " " << out.refcount;
          out = new_out.clone();
        } else {
          out = new_out;
        }

      } // im_in
    }  // inputs
    VLOG(3) << name << " update: " <<  out.refcount << " " << out_old.refcount;
  
    return true;
  }

  bool ImageNode::draw(float scale) 
  {
    Node::draw(scale);

    if (!out.empty()) {

      cv::Size sz = cv::Size(out.size().width * scale, out.size().height * scale);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC3);
      //cv::resize(tmp->get(), thumbnail, thumbnail.size(), 0, 0, cv::INTER_NEAREST );
      cv::resize(out, thumbnail, sz); //, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );
      cv::Mat graph_roi = graph(cv::Rect(loc.x, loc.y, sz.width, sz.height));
      graph_roi = cv::Scalar(0, 0, 255);
      thumbnail.copyTo(graph_roi);
    }
  }

  ////////////////////////////////////////////////////////////
  Webcam::Webcam()
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
    boost::thread cam_thread(&Webcam::runThread,this);

    // wait for single frame so there is a sample with the correct size
    while(!is_thread_dirty) {}
  }

  void Webcam::runThread()
  {
    do_capture = true;
    while(true) {
      if (do_capture) {
        /// TBD need to make this in separate thread
        if( !capture.grab() )
        {
          LOG(ERROR) << name << " Can not grab images." << endl;
          //return false;
        } 

        cv::Mat new_out;
        capture.retrieve(new_out);

        if (new_out.empty()) {
          LOG(ERROR) << name << " new image empty";
          //return false;
        }
        // I think opencv is reusing a mat within capture so have to clone it

        //if (&new_out.data == &out.data) {
        //
        cv::Mat tmp = new_out.clone();
        //out_lock.lock();
        out = tmp;
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
    ImageNode::update();

      is_dirty = is_thread_dirty;
      is_thread_dirty = false;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // TBD subclasses of Node that are input/output specific, or make that general somehow?
  Signal::Signal() : Node()
  {
    vcol = cv::Scalar(0,128,255);
  }

  void Signal::setup(const float new_step, const float offset) 
  {
    value = offset;
    step = new_step;
    LOG(INFO) << "Signal " << value << " " << new_step;
  }
  
  bool Signal::update() 
  {
    if (!Node::update()) return false;

    // it wouldn't be hard to update these
    // even if they aren't in need of updating, but don't for now
    value += step;
    if (value > 1.0) value = 0.0;
    if (value < 0.0) value = 1.0;
    is_dirty = true;

    return true;
  }

  //////////////////////////////////////////////////
  Saw::Saw() : Signal()
  {
    vcol = cv::Scalar(0,90,255);
  }
  
  void Saw::setup(const float new_step, const float offset) 
  {
    Signal::setup(new_step, offset);
  }

  bool Saw::update()
  { 
    if (!Node::update()) return false;

    value += step;
    if (value > 1.0) {
      step = -abs(step);
      value = 1.0;
    }
    if (value < 0.0) {
      step = abs(step);
      value = 0.0;
    }
    is_dirty = true;

    //LOG(INFO) << step << " " << value;
    return true;
  }

  //////////////////////////////////
  Buffer::Buffer() : ImageNode() {
    //this->max_size = max_size;
    //LOG(INFO) << "new buffer max_size " << this->max_size;
    vcol = cv::Scalar(200, 30, 200);
  }
 
  bool Buffer::update()
  {
    bool rv = ImageNode::update();

    if (!rv) return false;

    for (int i = 0; i < inputs.size(); i++) {
      
      ImageNode* im_in = dynamic_cast<ImageNode*> (inputs[i]);
      if (im_in) // && im_in->is_dirty) // TBD this produces flickering
        add(im_in->get()); 
    }

    return true;
  }

  void Buffer::add(cv::Mat new_frame)
  {
    if (new_frame.empty()) {
      LOG(ERROR) << name << " new_frame is empty";
      return;// TBD LOG(ERROR)
    }
    // TBD do clone here if frame is same
    frames.push_back(new_frame);
    out = frames[0];

    while (frames.size() >= max_size) frames.pop_front();
   
    VLOG(3) << name << " sz " << frames.size();
    
    // TBD is_dirty wouldn't be true for callers that want frames indexed from beginning if no pop_front has been done.
    is_dirty = true;
  }
  
  cv::Mat Buffer::get() {
    return ImageNode::get();
  }

  // not the same as the inherited get on purpose
  // many callers per time step could be calling this
  cv::Mat Buffer::get(const float fr) 
  {
    if (frames.size() < 1) {
      VLOG(1) << "no frames returning gray";
      cv::Mat tmp = cv::Mat(640, 480, CV_8UC3);
      tmp = cv::Scalar(128);
      return tmp;
    }

    int ind = (int)(fr*(float)frames.size());
    if (fr < 0) {
      ind = frames.size() - ind;
    }
    
    if (ind > frames.size() -1) ind = frames.size()-1;
    if (ind < 0) ind = 0;
    //ind %= frames.size();
   
    //VLOG_EVERY_N(1,10) 
    //LOG_EVERY_N(INFO, 10) << ind << " " << frames.size();
    return frames[ind];
  }

  // TBD get(int ind), negative ind index from last


///////////////////////////////////////////////////////////
  Tap::Tap() : ImageNode()
  {
    vcol = cv::Scalar(100, 30, 100);
  }

  void Tap::setup(Signal* new_signal, Buffer* new_buffer) 
  {
    signal = new_signal;
    buffer = new_buffer;
    
    inputs.clear();
    inputs.push_back(signal);
    inputs.push_back(buffer);
  }

  bool Tap::update()
  {
    if (!Node::update()) return false;

    if (is_dirty) {
      out = buffer->get(signal->value);
    }

    return true;
  }

  // this is sort of strange, maybe should have another object that can be many to one with the buffer?
  /*
   * cv::Mat get()
  {
    //if (rv.empty())
    if (VLOG_IS_ON(2)) {
    cv::line(rv, cv::Point(0,0), cv::Point( rv.cols, 0), cv::Scalar(0,0,0), 2);
    cv::line(rv, cv::Point(0,0), cv::Point( signal->value* rv.cols, 0), cv::Scalar(255,0,0), 2);
    }

    return out;
  }*/

 
  Add::Add() : ImageNode()
  {
    vcol = cv::Scalar(200, 200, 50);
  }
  
  void Add::setup(ImageNode* np1, ImageNode* np2, float nf1, float nf2) 
  {
    p1 = np1;
    p2 = np2;

    f1 = nf1;
    f2 = nf2;
   
    inputs.push_back(p1);
    inputs.push_back(p2);
  }

  bool Add::update()
  {
    if (!Node::update()) return false;

    VLOG(2) << "name " << is_dirty << " " << p1->name << " " << p2->name;
    if (is_dirty) {
      // TBD accomodate bad mats somewhere
      out = p1->get() * f1 + p2->get() * f2;
    }

    return true;
  }


}  // namespace bm

