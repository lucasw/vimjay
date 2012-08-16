
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
//#include <pair>

#include "nodes.h"
#include "camthing.h"

using namespace cv;
using namespace std;

namespace bm {

  //////////////////////////////////////////////////////////////////////////////////////////
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

    vector<Node*> input_vec = getInputVector();
    for (int i = 0; i < input_vec.size(); i++)
      input_vec[i]->setUpdate(); 
    
    return true;
  }

  bool Node::isDirty(const void* caller, const int ind, const bool clear) 
  {
    VLOG(4) << name << " " << this << " isDirty " << caller 
        << " " << ind << " " << clear;

    // first stage
    map<const void*, map<int, bool> >::iterator caller_map;  
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
    for (map<const void*, map<int, bool> >::iterator it = dirty_hash.begin(); it != dirty_hash.end(); it++) {
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

    vector<Node*> input_vec = getInputVector();
    for (int i = 0; i < input_vec.size(); i++)
    {
      input_vec[i]->update();
      if (input_vec[i]->isDirty(this, 0)) inputs_dirty = true;
    }

    // the inheriting object needs to set is_dirty as appropriate if it
    // isn't sufficient to have inputs determine it(e.g. it is sourcing change)
    if ( inputs_dirty ) {
      setDirty();
    }
    
    VLOG(4) << name << " in sz " << inputs.size() << " inputs dirty" << inputs_dirty;

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
      // can't use getInputVector because of this, and reference to strings
      cv::putText(graph, it->first, loc - cv::Point(-10,-ht*j - ht/2), 1, 1, cv::Scalar(100,255,245));
      j++;
      for (map<string, Node*>::iterator it2 = it->second.begin(); 
          it2 != it->second.end(); it2++) 
      {
        
        const string type = it->first;
        const string port = it2->first;

        cv::Point dst = loc - cv::Point(-10, -ht*j - ht/2);

        if (draw_selected_port && (selected_type == type) && (selected_port == port)) {

          VLOG(3) << name << " port " << draw_selected_port << " " << selected_type << " " << type << ", " 
            << selected_port << " " << port;

          // TBD need to draw a line from the source node to this point too
          cv::rectangle(graph, dst, dst + cv::Point(port.size()*8, -ht), 
              cv::Scalar(80, 80, 80), CV_FILLED );
          //cv::line( graph, dst, dst + cv::Point(20,0), cv::Scalar(0, 121, 100), 2, 4 );
        }

        cv::putText(graph, port, dst, 1, 1, cv::Scalar(255,100,245));
        j++;
        
        if (!it2->second) continue;
        
        cv::Point src = it2->second->loc + cv::Point(30,0);
        cv::Point mid = src + (dst - src) * 0.8;
        cv::line( graph, src, mid, cv::Scalar(0, 128/fr, 0), 2, 4 );
        cv::line( graph, mid, dst, cv::Scalar(0, 255/fr, 0), 2, CV_AA );

      } // for
    } // for

    cv::putText(graph, name, loc - cv::Point(10, ht), 1, 1, cv::Scalar(255,255,245));

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
  bool Node::getInputPort(
      //map<string, map< string, Node*> >& inputs,
      const string type, 
      const string port,
      Node*& rv)
  {
    rv = NULL;
    map<string, map<string, Node*> >::iterator image_map;  
    image_map = inputs.find(type);
    if (image_map == inputs.end()) {
      return false;
    }
     
    map<string, Node*>::iterator image_map2;  
    image_map2 = inputs[type].find(port);
    if (image_map2 == inputs[type].end()) return false;

    rv = image_map2->second;
    
    if (!rv) return false;

    return true;
  }

  // get an imagenode image from this nodes imagenode inputs
  bool Node::getImage(
    //map<string, map< string, Node*> >& inputs,
    const string port,
    cv::Mat& image,
    bool& is_dirty)
  {
    
    Node* nd = NULL;

    if (!getInputPort("ImageNode", port, nd)) 
      return false;
    
    VLOG(4) << name << " " << port << " " << nd;
    //if (require_dirty && !nd->isDirty()) return false;
    is_dirty = nd->isDirty(this, 20, false);

    ImageNode* im_in = dynamic_cast<ImageNode*> (nd);

    if (!im_in) return false;

    image = im_in->get();

    return true;
  }

  bool Node::getSignal(
    const string port, 
    float& val)
  {
    Node* nd;

    if (!getInputPort("Signal", port, nd)) return false;

    Signal* im_in = dynamic_cast<Signal*> (nd);

    if (!im_in) return false;

    val = im_in->value;

    return true;
  }

  bool Node::getBuffer(
    const std::string port,
    const float val,
    cv::Mat& image)
  {
    
    Node* nd;

    if (!getInputPort("Buffer", port, nd)) return false;

    Buffer* im_in = dynamic_cast<Buffer*> (nd);

    if (!im_in) return false;

    image = im_in->get(val);

    return true;
  }


  bool Node::getBuffer(
    const std::string port,
    const int val,
    cv::Mat& image)
  {
    
    Node* nd;

    if (!getInputPort("Buffer", port, nd)) return false;

    Buffer* im_in = dynamic_cast<Buffer*> (nd);

    if (!im_in) return false;

    image = im_in->get(val);

    return true;
  }


  void Node::printInputVector() 
  {

    for (map<string, map<string, Node*> >::iterator it = inputs.begin();
          it != inputs.end(); it++)
    {
      for (map<string, Node*>::iterator it2 = it->second.begin();
            it2 != it->second.end(); it2++)
      {
        LOG(INFO) << name << " inputVector " << it->first << " " << it2->first << " " << it2->second;
    }}
  }

  vector<pair< string, string > > Node::getInputStrings()
  {
    vector<pair< string, string > > rv;
    for (map<string, map<string, Node*> >::iterator it = inputs.begin();
          it != inputs.end(); it++)
    {
      for (map<string, Node*>::iterator it2 = it->second.begin();
            it2 != it->second.end(); it2++)
      {
        if (it2->second == NULL) continue;
          
        rv.push_back(pair<string,string> (it->first, it2->first));
      }
    }

    return rv;

  }

  // turn the double map inputs into a simple vector
  vector<Node*> Node::getInputVector()
  {
    vector<Node*> rv;

    for (map<string, map<string, Node*> >::iterator it = inputs.begin();
          it != inputs.end(); it++)
    {
      for (map<string, Node*>::iterator it2 = it->second.begin();
            it2 != it->second.end(); it2++)
      {
        if (it2->second == NULL) continue;
          
        rv.push_back(it2->second);
      }
    }

    return rv;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
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
     
    if (image_map->second.begin()->second == NULL) return true;

    ImageNode* im_in = dynamic_cast<ImageNode*> (image_map->second.begin()->second);

    // this 
    if (!im_in) {
      LOG(ERROR) << "wrong node attached to ImageNode input" 
          << image_map->second.begin()->second->name;
      return true;
    }

    //out_old = out;//.clone(); // TBD need to clone this?  It doesn't work
    cv::Mat new_out = im_in->get(); 

    out_old = out;
    if (new_out.refcount == out.refcount) {
      VLOG(4) << "dirty input is identical with old image " << new_out.refcount << " " << out.refcount;
      out = new_out.clone();
    } else {
      out = new_out;
    }
    
    VLOG(4) << name << " update: " <<  out.refcount << " " << out_old.refcount;
  
    return true;
  }

  bool ImageNode::draw(float scale) 
  {
    
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

      bool draw_thumb = true;
      if (loc.x + sz.width >= graph.cols) {
        LOG(ERROR) << name << " bad subregion " << loc.x << " " << sz.width << " " << graph.cols;
        draw_thumb = false;
      }
      if (loc.y + sz.height >= graph.rows) {
        LOG(ERROR) << name << " bad subregion " << loc.y << " " << sz.height << " " << graph.rows;
        draw_thumb = false;
      }

      if (draw_thumb) {
        cv::Mat graph_roi = graph(cv::Rect(loc.x, loc.y, sz.width, sz.height));
        graph_roi = cv::Scalar(0, 0, 255);
        thumbnail.copyTo(graph_roi);
      }
    }
    
    bool rv = Node::draw(scale);

    return rv;
  }

  bool Node::handleKey(int key)
  {
    return false;
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
  
  bool Signal::handleKey(int key)
  {
    bool valid_key = Node::handleKey(key);
    if (valid_key) return true;
    
    valid_key = true;

    if (key == ',') {
      value += abs(step);   
    }
    else if (key == 'm') {
      value -= abs(step);
    } else {
      valid_key = false;
    }
    
    if (valid_key) {
      VLOG(2) << value;
      setDirty();
    }
    
    return valid_key;
  }

  bool Signal::update() 
  {
    if (!Node::update()) return false;

    // it wouldn't be hard to update these
    // even if they aren't in need of updating, but don't for now
    value += step;
    if (value > 1.0) value = 0.0;
    if (value < 0.0) value = 1.0;

    VLOG(3) << "Signal " << name << " " << value;
    //is_dirty = true;
    setDirty();

    return true;
  }

  bool Signal::draw(float scale)
  {
    float x = (value)/(max-min);
    if ((max > 0) && (min > 0)) {
      x -= min/(max-min) + 0.1;
    }
    if ((max < 0) && (min < 0)) {
      x += max/(max-min) - 0.1;
    }

    cv::rectangle(graph, loc, 
        loc + cv::Point( x * 50.0 , 5), 
        cv::Scalar(255, 255, 100), CV_FILLED);

    return Node::draw(scale);
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

  
  //////////////////////////////////////////////////////////////////////////////////////////
  Buffer::Buffer() : ImageNode(), max_size(100) {
    //this->max_size = max_size;
    //LOG(INFO) << "new buffer max_size " << this->max_size;
    vcol = cv::Scalar(200, 30, 200);

    inputs["ImageNode"]["image"] = NULL;
  }
 
  bool Buffer::update()
  {
    bool rv = ImageNode::update();
    if (!rv) return false;
    
    bool im_dirty;
    if (!getImage("image", tmp, im_dirty)) return false;
    if (tmp.empty()) return false; 
    if (!im_dirty) return true;

    add(tmp); 

    if (frames.size() <= 0) return false;
    
    out = frames[0];

    return true;
  }

  bool Buffer::draw(float scale) 
  {

    // draw some grabs of the beginning frame, and other partway through the buffer 
    for (int i = 1; i < 4; i++) {
      int ind = i * frames.size() / 3;
      if (i == 3) ind = frames.size() - 1;
      if (ind >= frames.size())  continue;

      if (frames[ind].empty()) { 
        LOG(ERROR) << "frames " << i << CLERR << " is empty" << CLNRM;  continue; }

      if (out.empty()) out = frames[ind].clone();

      cv::Size sz = cv::Size(out.size().width * scale * 0.25, out.size().height * scale * 0.25);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC3);
      cv::resize(frames[ind], thumbnail, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );
      cv::Mat graph_roi = graph(cv::Rect(loc.x + i * out.cols*scale*0.25, loc.y + out.rows*scale, sz.width, sz.height));
      graph_roi = cv::Scalar(0, 0, 255);
      thumbnail.copyTo(graph_roi);
    }
    
    return ImageNode::draw(scale);
  }

  bool Buffer::add(cv::Mat new_frame, bool restrict_size)
  {
    if (new_frame.empty()) {
      LOG(ERROR) << name << CLERR << " new_frame is empty" << CLNRM;
      //is_dirty = false;
      return false;// TBD LOG(ERROR)
    }
    
    if ((frames.size() > 0) && 
        (new_frame.refcount == frames[frames.size()-1].refcount)) {
      new_frame = new_frame.clone();
      LOG_FIRST_N(INFO,10) << name << " cloning identical frame " 
          << new_frame.refcount << " " << frames[frames.size()-1].refcount;
      //return false;
    }

    frames.push_back(new_frame);
    
    if (restrict_size) {
      while (frames.size() >= max_size) frames.pop_front();
    }

    VLOG(4) << name << " sz " << frames.size();
    
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
    const int ind = (int)(fr * (float)frames.size());
    //if (fr < 0) {
    //  ind = frames.size() - ind;
    //}
    
    return get(ind);
  }

  cv::Mat Buffer::get(int ind)
  {
    if (frames.size() < 1) {
      VLOG(1) << "no frames returning gray";
      cv::Mat tmp = cv::Mat(640, 480, CV_8UC3);
      tmp = cv::Scalar(128);
      return tmp;
    }
    //if (ind > frames.size() - 1) ind = frames.size() - 1;
    //if (ind < 0) ind = 0;
    ind %= frames.size();
   
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
  //////////////////////////////////////////////////////////////////////////////////////////

}  // namespace bm

