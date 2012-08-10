
#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>
#include "boost/filesystem/operations.hpp"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>

#include <deque>
//#include <pair>

#include "nodes.h"
#include "camthing.h"

using namespace cv;
using namespace std;

namespace bm {


  ////////////////////////////////////////////////
  Node::Node() : enable(true) {
    do_update = false;

    //is_dirty = true;
    vcol = cv::Scalar(0,128,255);
  }

  // this finds all the nodes that are connected to this node and sets them to be updated
  bool Node::setUpdate()
  {
    // if do_update is already set return, this prevents infinite loops
    if (do_update) return false;

    do_update = true;

    for (map<string, map<string, Node*> >::iterator it = inputs.begin(); 
        it != inputs.end(); it++) 
    {
      for (map<string, Node*>::iterator it2 = it->second.begin(); 
          it2 != it->second.end(); it2++) 
        {
          if (it2->second)
            it2->second->setUpdate(); 
        }
    }
    
    return true;
  }

  bool Node::isDirty(void* caller, int ind, bool clear) 
  {
    // first stage
    map<void*, map<int, bool> >::iterator caller_map;  
    caller_map = dirty_hash.find(caller);

    if (caller_map == dirty_hash.end()) {
      dirty_hash[caller][ind] = false;
      return true;
    }
    
    // second stage
    map<int, bool>::iterator is_dirty;  
    is_dirty = caller_map->second.find(ind);
    if (is_dirty == caller_map->second.end()) {
      dirty_hash[caller][ind] = false;
      return true;
    }

    const bool rv = is_dirty->second;
    if (clear) {
      dirty_hash[caller][ind] = false;
    }
    return rv;
  }

  bool Node::setDirty()
  {
    for (map<void*, map<int, bool> >::iterator it = dirty_hash.begin(); it != dirty_hash.end(); it++) {
      for (map<int,bool>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++) {
        it2->second = true;
      }
    }

  }

  // the rv is so that an inheriting function will know whether to 
  // process or not
  bool Node::update() 
  {
    if (!do_update) return false;
    do_update = false; 

    // TBD should this occur before updating the inputs?
    if (!enable) return false;
    
    bool inputs_dirty = false;


    for (map<string, map<string, Node*> >::iterator it = inputs.begin(); 
        it != inputs.end(); it++) 
    {
      for (map<string, Node*>::iterator it2 = it->second.begin(); 
          it2 != it->second.end(); it2++) 
        {

        if (it2->second) {
          it2->second->update();
          if (it2->second->isDirty(this,0)) inputs_dirty = true;
        }

    }}


    // the inheriting object needs to set is_dirty as appropriate if it
    // isn't sufficient to have inputs determine it(e.g. it is sourcing change)
    if ( inputs_dirty ) {
      setDirty();
    }
    
    VLOG(2) << name << " in sz " << inputs.size() << " inputs dirty" << inputs_dirty;

    return true;
  }

  bool Node::draw(float scale) 
  {
    int fr = 1;
    if (!isDirty(this,1)) fr = 5;
    cv::Scalar col = cv::Scalar(vcol/fr);

    if (!enable) cv::circle(graph, loc, 10, cv::Scalar(0,0,100),-1);

    cv::circle(graph, loc, 20, col, 4);

    const int ht = 10;

    int j = 0;

    for (map<string, map<string, Node*> >::iterator it = inputs.begin(); 
        it != inputs.end(); it++) 
    {
      cv::putText(graph, it->name, loc - cv::Point(20,-ht*j), 1, 1, cv::Scalar(100,255,245));
      j++;
      for (map<string, Node*>::iterator it2 = it->second.begin(); 
          it2 != it->second.end(); it2++) 
      {
        cv::putText(graph, it2->name, loc - cv::Point(20,-ht*j), 1, 1, cv::Scalar(255,100,245));
        j++;

        if (!it2->second) continue;
        
        cv::Point src = it2->second->loc + cv::Point(20,0);
        cv::Point dst = loc - cv::Point(20, -ht*j);
        cv::Point mid = src + (dst - src) * 0.8;
        cv::line( graph, src, mid, cv::Scalar(0, 128/fr, 0), 2, 4 );
        cv::line( graph, mid, dst, cv::Scalar(0, 255/fr, 0), 2, CV_AA );

      }
    }

    cv::putText(graph, name, loc - cv::Point(0,5), 1, 1, cv::Scalar(255,255,245));

  }


  bool Node::load(cv::FileNodeIterator nd)
  {
    // TBD name, loc?
    (*nd)["enable"] >> enable;
  }

  bool Node::save(cv::FileStorage& fs)
  {
    string type = getId(this);

    fs << "typeid" << type;
    //fs << "typeid_mangled" << typeid(*all_nodes[i]).name();
    fs << "name" << name; 
    fs << "loc" << loc; 
    fs << "enable" << enable;
    //fs << "vcol" << p->vcol  ; 
    
  }
  
  //////////////////////////////////////////////////////

  /// TBD
  bool getNodeByNames(
      map<string, map< string, Node*> >& inputs,
      const string type, const string name,
      Node* rv)
  {
    map<string, map<string, Node*> >::iterator image_map;  
    image_map = inputs.find(type);
    if (image_map == inputs.end()) return false;
     
    map<string, Node*>::iterator image_map2;  
    image_map2 = inputs[type].find(name);
    if (image_map2 == inputs[type].end()) return false;

    rv = image_map2->second;
    
    if (!image_map2->second) return false;

    return true;
  }

  bool getImage(
    map<string, map< string, Node*> >& inputs,
    const string name,
    cv::Mat& image)
  {
    
    Node* nd;

    if (!getNodeByNames(inputs, "ImageNode", name, nd)) return false;

    ImageNode* im_in = dynamic_cast<ImageNode*> (nd);

    if (!im_in) return false;

    image = im_in->get();

    return true;
  }

  bool getSignal(
    map<string, map< string, Node*> >& inputs,
    const string name, 
    float& val)
  {
    Node* nd;

    if (!getNodeByNames(inputs, "Signal", name, nd)) return false;

    Signal* im_in = dynamic_cast<Signal*> (nd);

    if (!im_in) return false;

    image = im_in->value;

    return true;
  }

  bool getBuffer(
    map<string, map< string, Node*> >& inputs,
    const string name,
    const float val,
    cv::Mat& image)
  {
    
    Node* nd;

    if (!getNodeByNames(inputs, "Buffer", name, nd)) return false;

    Buffer* im_in = dynamic_cast<Buffer*> (nd);

    if (!im_in) return false;

    image = im_in->get(val);

    return true;
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
 
  /// Probably don't want to call this in most inheriting functions, skip back to Node::update()
  bool ImageNode::update() 
  {
    const bool rv = Node::update();
    if (!rv) return false;

    map<string, map<string, Node*> >::iterator image_map;  
    image_map = inputs.find("ImageNode");
    if (image_map == inputs.end()) return true;
     
    if (!image_map.begin()->second) return true;

    ImageNode* im_in = dynamic_cast<ImageNode*> (image_map.begin()->second);

    // this 
    if (!im_in) {
      LOG(ERROR) << "wrong node attached to ImageNode input" << image_map.begin()->second->name;
      return true;
    }

    //out_old = out;//.clone(); // TBD need to clone this?  It doesn't work
    cv::Mat new_out = im_in->get(); 

    out_old = out;
    if (new_out.refcount == out.refcount) {
      VLOG(2) << "dirty input is identical with old image " << new_out.refcount << " " << out.refcount;
      out = new_out.clone();
    } else {
      out = new_out;
    }
    
    VLOG(3) << name << " update: " <<  out.refcount << " " << out_old.refcount;
  
    return true;
  }

  bool ImageNode::draw(float scale) 
  {
    Node::draw(scale);

    tmp = out;
    if (!tmp.empty()) {

      cv::Size sz = cv::Size(tmp.size().width * scale, tmp.size().height * scale);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC3);
      //cv::resize(tmp->get(), thumbnail, thumbnail.size(), 0, 0, cv::INTER_NEAREST );
      cv::resize(tmp, thumbnail, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );
       
      int fr = 1;
      if (!isDirty(this,2)) fr = 5;
      cv::Scalar col = cv::Scalar(vcol/fr);

      cv::rectangle(graph, loc - cv::Point(2,2), loc + cv::Point(sz.width,sz.height) + cv::Point(2,2), col, CV_FILLED );
      if (loc.x + sz.width >= graph.cols) {
        LOG(ERROR) << name << " bad subregion " << loc.x << " " << sz.width << " " << graph.cols;
        return false;
      }
      if (loc.y + sz.height >= graph.rows) {
        LOG(ERROR) << name << " bad subregion " << loc.y << " " << sz.height << " " << graph.rows;
        return false;
      }

      cv::Mat graph_roi = graph(cv::Rect(loc.x, loc.y, sz.width, sz.height));
      graph_roi = cv::Scalar(0, 0, 255);
      thumbnail.copyTo(graph_roi);
    }
  }

  ////////////////////////////////////////////////////////////
  
  Rot2D::Rot2D()
  {
    angle = 0;
    scale = 1.0;
    center = cv::Point2f(0,0);
  }

  

  bool Rot2D::update()
  {
    if (!Node::update()) return false;

    // anything to rotate?
    if (inputs.size() < 1) return false;
    
    cv::Mat tmp_in;
    // "image" is the default image input name
    if (!getImage(inputs, "image", tmp_in)) return false;
    if (tmp_in.empty()) return false;

    getSignal(inputs, "angle", angle);     
    getSignal(inputs, "center_x", center.x);     
    getSignal(inputs, "center_y", center.y);

    //VLOG(1) << name << " " << is_dirty << " " << im_in->name << " " << im_in->is_dirty;

    cv::Mat rot = cv::getRotationMatrix2D(center, angle, scale);
    cv::warpAffine(tmp_in, tmp, rot, im_in->get().size(), INTER_NEAREST);

    out = tmp;
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
        new_out.convertTo(tmp, MAT_FORMAT,scale); //, 1.0/(255.0));//*255.0*255.0*255.0));

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

      if ( is_thread_dirty ) setDirty();
      is_thread_dirty = false;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // TBD subclasses of Node that are input/output specific, or make that general somehow?
  Signal::Signal() : Node()
  {
    vcol = cv::Scalar(0,128,255);
  }

  void Signal::setup(const float new_step, const float offset, const float min, const float max) 
  {
    value = offset;
    step = new_step;
    this->min = min;
    this->max = max;
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
    //is_dirty = true;
    setDirty();

    return true;
  }

  bool Signal::draw(float scale)
  {
    Node::draw(scale);

    cv::rectangle(graph, loc, 
        loc + cv::Point( (value - (max+min)/2.0)/(max-min) * 50.0 , 5), 
        cv::Scalar(255, 255, 100), CV_FILLED);

    return true;
  }

  bool Signal::load(cv::FileNodeIterator nd)
  {
    Node::load(nd);

    (*nd)["min"] >> min;
    (*nd)["max"] >> max;
    (*nd)["value"] >> value;
    (*nd)["step"] >> step;

  }

  bool Signal::save(cv::FileStorage& fs) 
  {
    Node::save(fs);

    fs << "min" << min;
    fs << "max" << max;
    fs << "value" << value;
    fs << "step" << step;
  }

  //////////////////////////////////////////////////
  Saw::Saw() : Signal()
  {
    vcol = cv::Scalar(0,90,255);
  }
  
  void Saw::setup(const float new_step, const float offset, const float min, const float max) 
  {
    Signal::setup(new_step, offset, min, max);
  }

  bool Saw::update()
  { 
    if (!Node::update()) return false;

    value += step;
    if (value > max) {
      step = -abs(step);
      value = max;
    }
    if (value < min) {
      step = abs(step);
      value = min;
    }
    setDirty();
    //is_dirty = true;

    //LOG(INFO) << step << " " << value;
    return true;
  }

  //////////////////////////////////
  Buffer::Buffer() : ImageNode(), max_size(100) {
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
      if (im_in && im_in->isDirty(this,3)) 
        add(im_in->get()); 
    }

    if (frames.size() <= 0) return false;
    
    out = frames[0];

    return true;
  }

  bool Buffer::draw(float scale) 
  {
    ImageNode::draw(scale);

    // draw some grabs of the beginning frame, and other partway through the buffer 
    for (int i = 1; i < 4; i++) {
      int ind = i * frames.size() / 3;
      if (i == 3) ind = frames.size() - 1;
      if (ind >= frames.size())  continue;

      if (frames[ind].empty()) { LOG(ERROR) << "frames " << i << " is empty";  continue; }

      cv::Size sz = cv::Size(out.size().width * scale * 0.25, out.size().height * scale * 0.25);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC3);
      cv::resize(frames[ind], thumbnail, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );
      cv::Mat graph_roi = graph(cv::Rect(loc.x + i * out.cols*scale*0.25, loc.y + out.rows*scale, sz.width, sz.height));
      graph_roi = cv::Scalar(0, 0, 255);
      thumbnail.copyTo(graph_roi);
    }
  }

  bool Buffer::add(cv::Mat new_frame, bool restrict_size)
  {
    if (new_frame.empty()) {
      LOG(ERROR) << name << " new_frame is empty";
      //is_dirty = false;
      return false;// TBD LOG(ERROR)
    }
    
    if ((frames.size() > 0) && 
        (new_frame.refcount == frames[frames.size()-1].refcount)) {
      new_frame = new_frame.clone();
      LOG(INFO) << name << " cloning identical frame " 
          << new_frame.refcount << " " << frames[frames.size()-1].refcount;
    }

    frames.push_back(new_frame);
    
    if (restrict_size) {
      while (frames.size() >= max_size) frames.pop_front();
    }

    VLOG(3) << name << " sz " << frames.size();
    
    // TBD is_dirty wouldn't be true for callers that want frames indexed from beginning if no pop_front has been done.
    setDirty();

    return true;
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
    
    if (ind > frames.size() - 1) ind = frames.size() - 1;
    if (ind < 0) ind = 0;
    //ind %= frames.size();
   
    //VLOG_EVERY_N(1,10) 
    //LOG_EVERY_N(INFO, 10) << ind << " " << frames.size();
    return frames[ind];
  }

  // TBD get(int ind), negative ind index from last

  bool Buffer::load(cv::FileNodeIterator nd)
  {
    ImageNode::load(nd);
    
    (*nd)["max_size"] >> max_size;
  }

  bool Buffer::save(cv::FileStorage& fs) 
  {
    ImageNode::save(fs);

    fs << "max_size" << max_size;
  }

///////////////////////////////////////////////////////////
  bool ImageDir::loadImages()
  {
    LOG(INFO) << name << " loading " << dir;

    boost::filesystem::path image_path(dir);
    if (!is_directory(image_path)) {
      LOG(ERROR) << name << " not a directory " << dir; 
      return false;
    }

    // TBD clear frames first?
    //
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    for (boost::filesystem::directory_iterator itr( image_path );
        itr != end_itr;
        ++itr )
    {
      if ( is_directory( *itr ) ) continue;
      
      stringstream ss;
      ss << *itr;
      string next_im = ( ss.str() );
      next_im = next_im.substr(1, next_im.size()-2);
      cv::Mat tmp0 = cv::imread( next_im );
     
      if (tmp0.data == NULL) { //.empty()) {
        LOG(WARNING) << name << " not an image? " << next_im;
        continue;
      }
      
      LOG(INFO) << name << " loaded image " << next_im;

      cv::Size sz = cv::Size(640,480);
      cv::resize(tmp0, tmp, sz, 0, 0, cv::INTER_NEAREST );

      const bool rv = add(tmp,false);
    }
    
    /// TBD or has sized increased since beginning of function?
    if (frames.size() == 0) {
      LOG(ERROR) << name << " no images loaded";
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
    inputs.push_back(new_signal);
    inputs.push_back(new_buffer);
  }

  bool Tap::update()
  {
    if (!Node::update()) return false;

    if (isDirty(this,4)) {
      float val = 0;
      getSignal(inputs, "val", val);     

      if (!getBuffer(inputs, "buf", val, tmp)) return false;
      
      out = tmp;
    }

    return true;
  }


  ///////////////////////////////////////////
  Add::Add() : ImageNode()
  {
    vcol = cv::Scalar(200, 200, 50);
  }
  
  void Add::setup(vector<ImageNode*> np, vector<float> nf) 
  {
    if (inputs.size() != nf.size()) {
      LOG(ERROR) << "mismatched inputs and coefficients";
      return; 
    }
    this->nf = nf; 
    
    inputs.resize(np.size());
    for (int i = 0; i < np.size(); i++) {
      inputs[0] = np[0];
    }
  }

  bool Add::update()
  {
    if (!Node::update()) return false;

    //VLOG(1) << "name " << is_dirty << " " << p1->name << " " << p1->is_dirty << ", " << p2->name << " " << p2->is_dirty ;
    if (isDirty(this, 5)) {
      // TBD accomodate bad mats somewhere

      cv::Size sz;

      bool done_something = false;
      
      // TBD should these vectors just be stored with some incrementing string?
      // TBD loop through all input ImageNodes and input signals (or vector values, need to be able
      // to handle either)
      for (int i = 0; i < inputs.size() && i < nf.size(); i++) { 
        ImageNode* in = dynamic_cast<ImageNode*>( inputs[i] );
        if (!in) continue; // TBD error
      
        if (in->get().empty()) continue; // TBD error

        if (!done_something) {
          out = in->get() * nf[i];
          sz = in->get().size();
          done_something = true;
        } else { 

          if (sz != in->get().size()) {
            LOG(ERROR) << name << " size mismatch " << sz.width << " " << sz.height << " != " << in->get().size().width << " " << in->get().size().height ;
            continue;
          }

          out += in->get() * nf[i];

        }
      }
    }

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

    for (int i = 0; i < (*nd)["nf"].size(); i++) {
      float new_coeff;
      (*nd)["nf"][i] >> new_coeff;
      nf.push_back(new_coeff);
    }
  }

  bool Add::save(cv::FileStorage& fs)
  {
    ImageNode::save(fs);

    fs << "nf" << "[:";
    for (int i = 0; i < nf.size(); i++) { 
      fs << nf[i]; 
    }
    fs << "]";
  }

}  // namespace bm

