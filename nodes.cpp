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


#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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
      // TBD may wan to track per output dirtiness later?
      if (input_vec[i]->isDirty(this, 0)) inputs_dirty = true;
    }

    // the inheriting object needs to set is_dirty as appropriate if it
    // isn't sufficient to have inputs determine it(e.g. it is sourcing change)
    if ( inputs_dirty ) {
      setDirty();
    }

    // TBD loop through svals and imvals and run getImage on each
    // or could that be combined with looping through the getInputVector above?
    
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

    for (inputsType::iterator it = inputs.begin(); it != inputs.end(); it++) 
    {
      const string type = it->first;
      // can't use getInputVector because of this, and reference to strings
      cv::putText(graph, type, loc - cv::Point(-10,-ht*j - ht/2), 1, 1, cv::Scalar(100,255,245),2);
      j++;

      for (inputsItemType::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++) 
      {
        
        const string port = it2->first;

        cv::Point dst = loc - cv::Point(-10, -ht*j - ht/2);

        if (draw_selected_port && (selected_type == type) && (selected_port == port)) {

          VLOG(3) << name << " port " << draw_selected_port << " " << selected_type << " " << type << ", " 
            << selected_port << " " << port;

          // TBD need to draw a line from the source node to this point too
          cv::rectangle(graph, dst, dst + cv::Point(port.size()*8, -ht), 
              cv::Scalar(180, 80, 80), CV_FILLED);
          //cv::line( graph, dst, dst + cv::Point(20,0), cv::Scalar(0, 121, 100), 2, 4 );
        }

        stringstream port_info;
        port_info << port;
        if (type == "Signal") {
          float val = getSignal(port);
          port_info << " " << val;
        }

        cv::putText(graph, port_info.str(), dst + cv::Point(1,0), 1, 1, cv::Scalar(75,50,75), 1);
        cv::putText(graph, port_info.str(), dst, 1, 1, cv::Scalar(255,100,245), 1);
        j++;
       
        Node* it_node = it2->second.first;
        string src_port = it2->second.second;
        if (!it_node) continue;
        
        cv::Point src = it_node->loc + cv::Point(100,-8);
        cv::line( graph, it_node->loc + cv::Point(0,-8), src, cv::Scalar(0, 80/fr, 0), 2, 1 );
        cv::Point mid = src + (dst - src) * 0.8;
        cv::line( graph, src, mid, cv::Scalar(0, 128/fr, 0), 2, 4 );
        cv::putText(graph, src_port, src, 1, 1, cv::Scalar(255,100,245), 1);
        cv::line( graph, mid, dst, cv::Scalar(0, 255/fr, 0), 2, CV_AA );

      } // for
    } // for

    cv::putText(graph, name, loc - cv::Point(9, ht), 1, 1, cv::Scalar(115,115,115));
    cv::putText(graph, name, loc - cv::Point(10, ht), 1, 1, cv::Scalar(255,255,255));

  }

  bool Node::load(cv::FileNodeIterator nd)
  {
    // TBD name, loc?
    (*nd)["enable"] >> enable;


    // blanket loading of all svals
    FileNode nd_in = (*nd)["svals"]; 
    if (nd_in.type() != FileNode::SEQ) {
      //LOG(ERROR) << "no nodes";
      //return false;
    }

    for (cv::FileNodeIterator it = nd_in.begin(); it != nd_in.end(); ++it) {
      float val;
      const string sval_name = (*it)["name"];
      (*it)["value"] >> val;
      setSignal( sval_name, val);
      LOG(INFO) << name << " : " << sval_name << " = " << val;
      /* this doesn't work, name returns blank
       * though there are a matching number of entries to sval items 
       * */
      /*
      const string sval_name = (*it).name();
      (*it)[sval_name] >> val;
      setSignal( sval_name, val);
      LOG(INFO) << name << " " << (*it) << " " << sval_name << " " << val;
      */
    }

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
   
    fs << "svals" << "[";
    // this takes care of all saving of svals, loading isn't so elegant though?
    for (map<string, float >::iterator it = svals.begin(); it != svals.end(); it++) {
      fs << "{";
      //fs << it->first << it->second;  // this is clever but I don't know how to load it
      fs << "name" << it->first;
      fs << "value" << it->second;
      fs << "}";
    }
    fs << "]";

  }
  
  bool Node::handleKey(int key)
  {
    return false;
  }

  //////////////////////////////////////////////////////

  /// TBD
  bool Node::getInputPort(
      //map<string, map< string, Node*> >& inputs,
      const string type, 
      const string port,
      Node*& rv,
      string& src_port)
  {
    rv = NULL;
    inputsType::iterator image_map;  
    image_map = inputs.find(type);
    if (image_map == inputs.end()) {
      return false;
    }
     
    inputsItemType::iterator image_map2;  
    image_map2 = inputs[type].find(port);

    if (image_map2 == inputs[type].end()) return false;

    rv = image_map2->second.first;
    src_port = image_map2->second.second;
    
    //if (!rv) return false;

    return true;
  }

  void Node::setInputPort(
      const std::string type, 
      const std::string port,
      const Node* rv,
      const std::string src_port
    )
  {
    // this will create the entries if they don't alread exists, without clobbering their values
    // TBD only do this if required?
    if ((type == "ImageNode") || (type == "ImageOut"))
      getImage(port);
    if (type == "Signal")
      getSignal(port);
    
    inputs[type][port] = std::pair<Node*, string> ((Node*)rv, src_port);
    VLOG(1) << name << " setInputPort: " << type << " " << port << " from "
      << CLTXT /*<< inputs[type][port].first->name << " "*/  // not necessarily non-NULL
        << inputs[type][port].second << CLNRM; 
  }

  // get an imagenode image from this nodes imagenode inputs
  cv::Mat Node::getImage(
    //map<string, map< string, Node*> >& inputs,
    const string port,
    bool& valid)
    //bool& is_dirty)
  {
 
    string type = "ImageNode";
    if (port.substr(0,3) == "out") {
      type = "ImageOut";
    }
  
    valid = false;
    cv::Mat im = imvals[port];
    Node* nd = NULL;
    string src_port = "";

    if (!getInputPort(type, port, nd, src_port)) 
      return im;
   
    if (!nd) return im;

    VLOG(4) << name << " " << port << " " << nd << " " << src_port;
    
    // this could result in infinite loops, need isDirty protection
    if (!nd->isDirty(this, 20)) return im;

    cv::Mat im2 = nd->getImage(src_port);
    if (im2.empty()) return im;

    imvals[port] = im2;
    // TBD setDirty? No don't
    
    valid = true;

    return im2;
  }

  
  bool Node::setImage(const std::string port, cv::Mat& im)
  {
    // can't set the image if it is controlled by an input node

    string type = "ImageNode";
    if (port.substr(0,3) == "out") {
      type = "ImageOut";
    }

    Node* nd;
    string src_port;
    if (getInputPort(type, port, nd, src_port)) {
      if (nd != NULL) return false;
    } else {
      //pair<Node*, string> in_pair;
      //in_pair.first = NULL;
      //in_pair.second = "";
      //inputs[type][port] = in_pair;
      inputs[type][port] = std::pair<Node*, string> (NULL, "");
    }
  
    imvals[port] = im;
    return true;
  }
  
  float Node::getSignal(
    const string port, 
    bool& valid)
  {
    valid = false; 
    // first try non-input node map
    float val = svals[port];
    Node* nd = NULL;
    string src_port;
    // then look at input nodes
    if (!getInputPort("Signal", port, nd, src_port)) {
      // create it if it doesn't exist
      VLOG(1) << "creating " << CLTXT <<  name << " " << port << CLNRM;
      inputs["Signal"][port] = std::pair<Node*, string> (NULL, "");
      return val;
    }

    if (!nd) return val;
   
    // prevent loops
    if (!nd->isDirty(this, 22)) {
      return val;
    }
    
    float new_val = nd->getSignal(src_port,valid);

    //VLOG(1) << name << " " << src_port << " " << valid << " " << new_val << " " << val;
    //if (!valid) return val;
    
    val = new_val;
    // store a copy here in case the input node is disconnected
    svals[port] = val;
    // TBD setDirty?  Probably shouldn't, would defeat the isDirty loop prevention
    valid = true;
    return val;
  }

  bool Node::setSignal(const std::string port, float val)
  {
    Node* nd;
    string src_port;
    if (getInputPort("Signal", port, nd, src_port)) {
      if (nd != NULL) return false;
    } else {
      // create it since it doesn't exist
      inputs["Signal"][port] = std::pair<Node*, string> (NULL, "");
    }

    svals[port] = val;
  }

  bool Node::getBuffer(
    const std::string port,
    const float val,
    cv::Mat& image)
  {
    
    Node* nd;
    string src_port;
    if (!getInputPort("Buffer", port, nd, src_port)) return false;

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
    string src_port;
    if (!getInputPort("Buffer", port, nd, src_port)) return false;

    Buffer* im_in = dynamic_cast<Buffer*> (nd);

    if (!im_in) return false;

    image = im_in->get(val);

    return true;
  }

  //typedef map<string, map<string, pair<Node*, string> > >::iterator inputsIter;

  void Node::printInputVector() 
  {

    for (inputsType::iterator it = inputs.begin(); it != inputs.end(); it++)
    {
      for (inputsItemType::iterator it2 = it->second.begin();
            it2 != it->second.end(); it2++)
      {
        LOG(INFO) << name << " inputVector " << it->first << " " << it2->first << " " 
            << it2->second.first << " " << it2->second.second;
    }}
  }

  vector<pair< string, string > > Node::getInputStrings()
  {
    vector<pair< string, string > > rv;
    for (inputsType::iterator it = inputs.begin(); it != inputs.end(); it++)
    {
      for (inputsItemType::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
      {
        //if (it2->second.first == NULL) continue;
          
        rv.push_back(pair<string,string> (it->first, it2->first));
      }
    }

    return rv;

  }

  // turn the double map inputs into a simple vector
  vector<Node*> Node::getInputVector()
  {
    vector<Node*> rv;

    for (inputsType::iterator it = inputs.begin(); it != inputs.end(); it++)
    {
      for (inputsItemType::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
      {
        if (it2->second.first == NULL) continue;
          
        rv.push_back(it2->second.first);
      }
    }

    return rv;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  ImageNode::ImageNode() : Node()
  {
    vcol = cv::Scalar(255,0,255);
  
    // create the entry for out
    // TBD get default resolution from somewhere (a singleton registry?)
    // TBD should out really be in the imvals, since it is an output not an input?
    cv::Mat tmp;
    setImage("out", tmp);
  }
   

 
  /// Probably don't want to call this in most inheriting functions, skip back to Node::update()
  bool ImageNode::update() 
  {
    const bool rv = Node::update();
    if (!rv) return false;

    //setImage("out_old", ;
    // TBD any reason to call this here?
    cv::Mat in = getImage("in");
    if (in.empty()) return true; 
    
    setImage("out", in);
    setDirty();
  #if 0
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
    #endif

    //VLOG(4) << name << " update: " <<  out.refcount << " " << out_old.refcount;
  
    return true;
  }

  bool ImageNode::draw(float scale) 
  {
    
    // TBD if update is the only function to call getImage, then
    //  the imval will have been updated
    cv::Mat tmp = imvals["out"]; // getImage("out");
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

  bool ImageNode::writeImage()
  {
    LOG(INFO) << name << " writing ImageNode image to disk";

    if (dir_name.str() == "") {
      time_t t1 = time(NULL);

      // TBD define path to data somewhere to be reused by all
      dir_name << "../data/" << t1 << "_" << name;
      LOG(INFO) << name << " creating directory" << dir_name.str();
    }
    
    boost::filesystem::create_directories(dir_name.str());

    stringstream file_name;
    int write_count = getSignal("write_count");
    file_name << dir_name.str() << "/image_" << (write_count + 1000000) << ".png";

    cv::Mat out = imvals["out"]; //getImage("out");
    if (out.empty()) return false;

    cv::imwrite(file_name.str(), out);
    write_count++;
    setSignal("write_count", write_count);
    // TBD register that these frames have been saved somewhere so it is easy to load
    // them up again?
  }

  bool ImageNode::handleKey(int key)
  {
    bool valid_key = true;
    
    if (key == 'p') {
      writeImage(); 
    } 
    else {
      valid_key = false;
    }

    return valid_key;
  } 
  //////////////////////////////////////////////////////////////////////////////////////////
  // TBD subclasses of Node that are input/output specific, or make that general somehow?
  Signal::Signal() : Node()
  {
    vcol = cv::Scalar(0,128,255);

    setSignal("min", 0);
    setSignal("max", 1);
    setSignal("value", 0);
    setSignal("step", 0.1);
  }

  void Signal::setup(const float new_step, const float offset, const float min, const float max) 
  {
    setSignal("min", min);
    setSignal("max", max);
    setSignal("value", offset);
    setSignal("step", new_step);

    LOG(INFO) << "Signal " << offset << " " << new_step;
  }
  
  bool Signal::handleKey(int key)
  {
    bool valid_key = Node::handleKey(key);
    if (valid_key) return true;
    
    valid_key = true;

    float value = getSignal("value");

    if (key == ',') {
      value += abs(getSignal("step"));   
    }
    else if (key == 'm') {
      value -= abs(getSignal("step"));
    } else {
      valid_key = false;
    }
    
    if (valid_key) {
      VLOG(2) << value;
      setSignal("value", value);
      setDirty();
    }
    
    return valid_key;
  }

  bool Signal::update() 
  {
    if (!Node::update()) return false;

    float step = getSignal("step");
    float min = getSignal("min");
    float max = getSignal("max");
    float value = getSignal("value");
    // it wouldn't be hard to update these
    // even if they aren't in need of updating, but don't for now
    value += step;
    if (value > max) value = max;
    if (value < min) value = min;
    
    setSignal("value", value);

    VLOG(2) << "Signal " << name << " " << value;
    //is_dirty = true;
    setDirty();

    return true;
  }

  bool Signal::draw(float scale)
  {
    float step = getSignal("step");
    float min = getSignal("min");
    float max = getSignal("max");
    float value = getSignal("value");
    
    float x = (value)/(max - min);
    if ((max > 0) && (min > 0)) {
      x -= min/(max - min) + 0.1;
    }
    if ((max < 0) && (min < 0)) {
      x += max/(max - min) - 0.1;
    }

    cv::rectangle(graph, loc, 
        loc + cv::Point( x * 50.0 , 5), 
        cv::Scalar(255, 255, 100), CV_FILLED);

    //stringstream sstr;
    //sstr << value << " " << min << " " << max << " " << step;
    //cv::putText(graph, sstr.str(), loc + cv::Point(20,-30), 1, 1, cv::Scalar(200,200,200));
    
    return Node::draw(scale);
  }

  bool Signal::load(cv::FileNodeIterator nd)
  {
    Node::load(nd);

    // TBD if svals has all the values, could just loop through it in Node::load
   /* (*nd)["min"] >> min;
    (*nd)["max"] >> max;
    (*nd)["value"] >> value;
    (*nd)["step"] >> step;
    */
  }

  bool Signal::save(cv::FileStorage& fs) 
  {
    Node::save(fs);

    /*
    fs << "min" << min;
    fs << "max" << max;
    fs << "value" << value;
    fs << "step" << step;
    */
  }

  
  //////////////////////////////////////////////////////////////////////////////////////////
  Buffer::Buffer() : ImageNode()
  {
    //this->max_size = max_size;
    //LOG(INFO) << "new buffer max_size " << this->max_size;
    vcol = cv::Scalar(200, 30, 200);

    //setImage("image", cv::Mat());
    setSignal("max_size", 100);
    cv::Mat tmp;
    setImage("in", tmp);
  }
 
  bool Buffer::update()
  {
    bool rv = Node::update(); // ImageNode::update();
    if (!rv) return false;
   
    if (!isDirty(this,21)) { return true;}
   
    float val = getSignal("max_size");
    max_size = val;
    if (max_size < 1) max_size = 1;
   
    // TBD this is kind of confusing, out is being used as 
    // and input and an output
    cv::Mat in = getImage("in");
    if (!in.empty()) 
      add(in); 

    if (frames.size() <= 0) return false;
    
    cv::Mat out = frames[0];
    setImage("out", out);

    if (VLOG_IS_ON(5)) {
      VLOG(15) << frames.size();
      imshow(name, out);
    }

    return true;
  }

  bool Buffer::draw(float scale) 
  {
    cv::Mat out = getImage("out");
    // draw some grabs of the beginning frame, and other partway through the buffer 
    for (int i = 1; i < 4; i++) {
      int ind = i * frames.size() / 3;
      if (i == 3) ind = frames.size() - 1;
      if (ind >= frames.size())  continue;

      if (frames[ind].empty()) { 
        LOG(ERROR) << "frames " << i << CLERR << " is empty" << CLNRM;  continue; }

      if (out.empty()) out = frames[0];//.clone();

      cv::Size sz = cv::Size(out.size().width * scale * 0.25, out.size().height * scale * 0.25);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC3);
      cv::resize(frames[ind], thumbnail, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );
      cv::Mat graph_roi = graph(cv::Rect(loc.x + i * out.cols*scale*0.25, loc.y + out.rows*scale, sz.width, sz.height));
      graph_roi = cv::Scalar(0, 0, 255);
      thumbnail.copyTo(graph_roi);
    }

    if (VLOG_IS_ON(9)) {
      for (int i = 0; i < frames.size() ; i++) {
        imshow(name + boost::lexical_cast<string>(i), frames[i]);
      }
    }
    
    if (frames.size() < getSignal("max_size"))
      vcol = cv::Scalar(200, 30, 200);
    else
      vcol = cv::Scalar(55, 255, 90);
    
    return ImageNode::draw(scale);
  }

  bool Buffer::add(cv::Mat& new_frame, bool restrict_size)
  {
    if (new_frame.empty()) {
      LOG(ERROR) << name << CLERR << " new_frame is empty" << CLNRM;
      //is_dirty = false;
      return false;// TBD LOG(ERROR)
    }
    
    if ((frames.size() > 0) && 
        (new_frame.refcount == frames[frames.size()-1].refcount)) {
      new_frame = new_frame.clone();
      LOG_FIRST_N(INFO,15) << name << " cloning identical frame " 
          << new_frame.refcount << " " << frames[frames.size()-1].refcount;
      //return false;
    }

    frames.push_back(new_frame);
    
     

    if (restrict_size) {
      while (frames.size() > max_size) frames.pop_front();
    }

    VLOG(4) << name << " sz " << frames.size();
    
    // TBD is_dirty wouldn't be true for callers that want frames indexed from beginning if no pop_front has been done.
    setDirty();

    return true;
  }
  
  cv::Mat Buffer::get() {
    return get(0);
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
  
    VLOG(2) << name << " ind " << ind;

    //VLOG_EVERY_N(1,10) 
    //LOG_EVERY_N(INFO, 10) << ind << " " << frames.size();
    return frames[ind];
  }

  // TBD get(int ind), negative ind index from last
 
  /*
   * Write all the frames to disk
   */
  bool Buffer::writeImage()
  {
    time_t t1 = time(NULL);

    stringstream dir_name;
    // TBD define path to data somewhere to be reused by all
    dir_name << "../data/" << t1 << "_" << name;
    
    boost::filesystem::create_directories(dir_name.str());
    LOG(INFO) << name << " writing Buffer images to disk " << CLTXT << dir_name.str() << CLNRM;

    // TBD move to another thread
    for (int i = 0; i < frames.size(); i++) {
      stringstream file_name;
      file_name << dir_name.str() << "/buffer_" << (i + 1000000) << ".jpg";
      cv::imwrite(file_name.str(), frames[i]);
     
      int write_count = getSignal("write_count");
      write_count++;
      setSignal("write_count", write_count);
    }
    // TBD register that these frames have been saved somewhere so it is easy to load
    // them up again?
  }

  bool Buffer::load(cv::FileNodeIterator nd)
  {
    ImageNode::load(nd);
    
    //(*nd)["max_size"] >> max_size;
    //svals["max_size"] = max_size;
  }

  bool Buffer::save(cv::FileStorage& fs) 
  {
    ImageNode::save(fs);

    //fs << "max_size" << max_size;
  }
  //////////////////////////////////////////////////////////////////////////////////////////

}  // namespace bm

