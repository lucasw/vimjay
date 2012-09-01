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

  Connector::Connector() :
    parent(NULL),
    src(NULL),
    writeable(true),
    type(SIGNAL),
    value(0),
    name(""),
    is_dirty(true)
  {

  }

  bool Connector::update()
  {
    is_dirty = set_dirty;
    set_dirty = false;
    if (src) {
      src->parent->update();
      if (src->parent->isDirty(this, 0)) {
        is_dirty = true;
        
        // now get dirtied data
        value = src->value;
        im = src->im;
      }
    } 
  }

  void Connector::draw(cv::Mat graph) 
  {
   
    if (src) {
      vector<cv::Point2f> control_points;
      control_points.resize(4);
      control_points[0] = src->parent->loc + src->loc + cv::Point(20,0);
      control_points[3] = parent->loc + loc;
    
      cv::Point2f diff = control_points[3] - control_points[0];
      float dist = abs(diff.x) + abs(diff.y);

      control_points[1] = control_points[0] + cv::Point2f(dist/3.0,0);
      control_points[2] = control_points[3] - cv::Point2f(dist/3.0,0);
      getBezier(control_points, connector_points, 20);

      for (int i = 1; i < connector_points.size(); i++) {
        cv::Scalar col;
        const float fr = (float)i/(float)connector_points.size();
        col = cv::Scalar(255*fr, 128+128*fr, 255);
        cv::line(graph, connector_points[i-1], connector_points[i], col, 2, CV_AA ); 
      }
    }

    if (highlight) {
      cv::rectangle(graph, 
          parent->loc + loc, 
          parent->loc + loc + cv::Point(name.size()*8, -10), 
          cv::Scalar(180, 80, 80), CV_FILLED);
    }

    stringstream port_info;
    port_info << name;

    // color based on type late
    cv::Scalar col = cv::Scalar(200,200,200);
    if (type == SIGNAL) {
      col = cv::Scalar(55,255,255);
      port_info << " " << value;
    } else if (type == IMAGE) {
      col = cv::Scalar(255,55,255);
    } else if (type == BUFFER) {
      col = cv::Scalar(255,255,55);
    }
    
    cv::putText(graph, port_info.str(), parent->loc + loc, 1, 1, col, 1);

  }

  //////////////////////////////////////////////////////////////////////////////////////////
  Node::Node() : 
    enable(true),
    selected_type(NONE),
    selected_port_ind(-1),
    selected_port(""),
    do_update(false)
  {

    //is_dirty = true;
    vcol = cv::Scalar(200,200,200);
  }

/*
  Node::Node(string name, cv::Point loc, cv::Mat graph ) : 
    enable(true),
    selected_type(NONE),
    selected_port_ind(-1),
    name(name),
    loc(loc),
    graph(graph),
    do_update(false)
  {

    //is_dirty = true;
    vcol = cv::Scalar(0,128,255);
  }
  */

  // this finds all the nodes that are connected to this node and sets them to be updated
  bool Node::setUpdate()
  {
    // if do_update is already set return, this prevents infinite loops
    if (do_update) return false;

    do_update = true;

    for (int i = 0; i < ports.size(); i++) {
      if (ports[i]->src) ports[i]->src->parent->setUpdate();
    }
    
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

    for (int i = 0; i < ports.size(); i++)
    {
      ports[i]->update();
        //ports[i]->src->parent->update();
        // TBD may wan to track per output dirtiness later?
      if (ports[i]->is_dirty) inputs_dirty = true;
    }

    // the inheriting object needs to set is_dirty as appropriate if it
    // isn't sufficient to have inputs determine it(e.g. it is sourcing change)
    if ( inputs_dirty ) {
      setDirty();
    }

    // TBD loop through ports and run getImage on each
    // or could that be combined with looping through the getInputVector above?
    
    VLOG(4) << name << " in sz " << ports.size() << " inputs dirty" << inputs_dirty;

    return true;
  }

  bool Node::draw() 
  {
    int fr = 1;
    if (!isDirty(this,1)) fr = 5;
    cv::Scalar col = cv::Scalar(vcol/fr);

    if (!enable) cv::circle(graph, loc, 10, cv::Scalar(0,0,100), -1);

    cv::circle(graph, loc, 24, col, 4);

    const int ht = 10;

    int j = 0;

    for (int i = 0; i < ports.size(); i++) {
      if (i == selected_port_ind) ports[i]->highlight = true;
      else ports[i]->highlight = false;
      ports[i]->draw(graph);
    }

    cv::putText(graph, name, loc - cv::Point(9, ht), 1, 1, cv::Scalar(115,115,115));
    cv::putText(graph, name, loc - cv::Point(10, ht), 1, 1, cv::Scalar(255,255,255));

  }

  bool Node::load(cv::FileNodeIterator nd)
  {
    // TBD name, loc?
    (*nd)["enable"] >> enable;

     #if 0 
    // blanket loading of all svals
    FileNode nd_in = (*nd)["inputs"]; 
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
    #endif
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
  
  bool Node::handleKey(int key)
  {
    bool valid_key = true;
  
    VLOG(1) << selected_type << " \"" << selected_port << "\"";
    if ((selected_type == SIGNAL) && (selected_port != "")) { 
      float value = getSignal(selected_port);

      if (key == '.') {
        value *= 0.9;   
      }
      else if (key == '/') {
        value *= 1.1;
      }
      else if (key == ',') {
        value += 1;   
      }
      else if (key == 'm') {
        value -= 1;
      }
      else if (key == 'n') {
        value = 0;
      } 
      else {
        valid_key = false;
      }

      if (valid_key) {
        VLOG(2) << value;
        setSignal(selected_port, value);
        setDirty();
      }
      return valid_key;
    }

    valid_key = false;

    return valid_key;
  }

  //////////////////////////////////////////////////////

  bool Node::getNextPort(const conType type)
  {
    
    for (int i = 0; i < ports.size(); i++) {
      int ind = (i + 1 + selected_port_ind) % ports.size();
      
      VLOG(3) << ind << " : " << type << " " << ports[ind]->type << ", " 
          << ports[ind]->name;

      if ((type == NONE) || (type == ports[ind]->type)) {
        selected_port_ind = ind;
        selected_port = ports[ind]->name;
        selected_type = ports[ind]->type;
        return true;
      }

    }

    return false;

  }

  /// TBD
  bool Node::getInputPort(
      const conType type, 
      const string port,
      Connector*& con,
      string& src_port)
  {
    con = NULL;
  
    for (int i = 0; i < ports.size(); i++) {

      VLOG(6) << i << " " << ports[i]->type << " " << ports[i]->name << ", " << type << " " << port;
      
      if ((ports[i]->type == type) && (ports[i]->name == port)) {
        con = ports[i];
        if (ports[i]->src) {
          src_port = ports[i]->src->name;
        }
        return true;        
      }

    }

    return false;
  }

  void Node::setInputPort(
      const conType type, 
      const std::string port,
      Node* src_node,
      const std::string src_port
    )
  {
    // this will create the entries if they don't alread exists, without clobbering their values
    // TBD only do this if requested?
   
    Connector* con;
    string existing_port;
    bool con_exists = getInputPort(type, port, con, existing_port);
    if (!con_exists) {
      con = new Connector();
      con->name = port;
      con->parent = this;
      con->type = type;
      con->loc = cv::Point(0, ports.size()*10);

      ports.push_back(con);

      VLOG(2) << "\"" << con->parent->name << "\"" <<CLTX2 << " new connector " << CLNRM 
          << type << " \"" << con->name << "\"";
    }

    Connector* src_con = NULL;
    if (src_node) {
      bool src_con_exists = src_node->getInputPort(type, src_port, src_con, existing_port);
      if (!src_con_exists) {
        // this will produce connectors in the src parent in an order determined
        // by the way the earlier connections use them as ports
        src_con = new Connector();
        src_con->name = src_port;
        src_con->parent = src_node;
        src_con->type = type;
        src_con->loc = cv::Point(0, src_node->ports.size()*10);
        
        src_node->ports.push_back(src_con);
        VLOG(2) << con->parent->name << " new src Connector " << type << " " << src_con->name; 
      }
      
      VLOG(1) << "\"" << name << "\" setInputPort: " << type << " " << port << " from "
        << CLTXT /*<< inputs[type][port].first->name << " "*/  // not necessarily non-NULL
        << "\"" << src_port << "\"" << CLNRM; 
    } else {

      VLOG(1) << "\"" << name << "\" setInputPort: " << type << " " << port;
    }

    con->src = src_con;
  }
  
  bool Node::setImage(const std::string port, cv::Mat& im)
  {
    // can't set the image if it is controlled by an input node

    //if (port.substr(0,3) == "out") {
    //  type = "ImageOut";
    //}

    Connector* con = NULL;
    string src_port;
    if (!getInputPort(IMAGE, port, con, src_port)) {
      // create it since it doesn't exist
      setInputPort(IMAGE, port, NULL, "");
      // now get it again TBD actually check rv
      if (!getInputPort(IMAGE, port, con, src_port)) {
        LOG(ERROR) << "still can't get port";
        return false;
      }
    }
     
    if (!con) {
      LOG(ERROR) << "no connector";
      return false;
    }

    // can't set signal if it is controlled by src port 
    if (con->src) return false;

    con->im = im;
    con->set_dirty = true;
  
    return true;
  }
 
  bool Node::setSignal(const std::string port, float val)
  {
    Connector* con = NULL;
    string src_port;
    if (!getInputPort(SIGNAL, port, con, src_port)) {
      // create it since it doesn't exist
      setInputPort(SIGNAL, port, NULL, "");
      // now get it again TBD actually check rv
      if (!getInputPort(SIGNAL, port, con, src_port)) {
        LOG(ERROR) << "still can't get connector";
        return false;
      }
    }
    
    // can't set signal if it is controlled by src port 
    if (con->src) return false;

    con->value = val;
    con->set_dirty = true;

    return true;
  }
 
 // get an imagenode image from this nodes imagenode inputs
  cv::Mat Node::getImage(
    const string port,
    bool& valid)
  {
    /*
    string type = "ImageNode";
    if (port.substr(0,3) == "out") {
      type = "ImageOut";
    }
    */
  
    valid = false;
    cv::Mat im;
    Connector* con = NULL;
    string src_port = "";

    if (!getInputPort(IMAGE, port, con, src_port)) {
      // create it if it doesn't exist
      VLOG(1) << name << " creating IMAGE " << CLTXT <<  name << " " << port << CLNRM;
      setInputPort(IMAGE, port, NULL, "");
      return im;
    }

    VLOG(4) << name << " " << port << " " << " " << src_port;
   
    im = con->im;
    valid = true;

    return im;
  }

  float Node::getSignal(
    const string port, 
    bool& valid)
  {
    valid = false; 
    // first try non-input node map
    
    Connector* con = NULL;
    string src_port;
    // then look at input nodes
    if (!getInputPort(SIGNAL, port, con, src_port)) {
      // create it if it doesn't exist
      VLOG(1) << name << " creating SIGNAL " << CLTXT <<  name << " " << port << CLNRM;
      setInputPort(SIGNAL, port, NULL, "");
      return 0;
    }

    if (!con) return 0;
    //VLOG(1) << name << " " << src_port << " " << valid << " " << new_val << " " << val;
    //if (!valid) return val;
    float val = con->value;
    
    // TBD setDirty?  Probably shouldn't, would defeat the isDirty loop prevention
    valid = true;
    return val;
  }

  cv::Mat Node::getBuffer(
    const std::string port,
    const float val)
  {
   
    cv::Mat tmp;
    Connector* con;
    string src_port;
    if (!getInputPort(BUFFER, port, con, src_port)) {
      return tmp;
    }
  
    // TBD Buffer is dissimilar to Image and Signals currently
    if (!con) return tmp;

    tmp = con->im;

    if ((!con->src) || (!con->src->parent)) return tmp;

    Buffer* im_in = dynamic_cast<Buffer*> (con->src->parent);

    if (!im_in) {
      LOG(ERROR) << name  << " " << port << " improper Buffer connected";
      return tmp;
    }

    cv::Mat image = im_in->get(val);
    con->im = image;

    return image;
  }


  cv::Mat Node::getBuffer(
    const std::string port,
    const int val)
    //cv::Mat& image)
  {
    
    cv::Mat tmp;
    Connector* con;
    string src_port;
    if (!getInputPort(BUFFER, port, con, src_port)) {
      return tmp;
    }
  
    // TBD Buffer is dissimilar to Image and Signals currently
    if (!con) return tmp;

    tmp = con->im;

    if ((!con->src) || (!con->src->parent)) return tmp;

    Buffer* im_in = dynamic_cast<Buffer*> (con->src->parent);

    if (!im_in) {
      LOG(ERROR) << name  << " " << port << " improper Buffer connected";
      return tmp;
    }

    cv::Mat image = im_in->get(val);
    con->im = image;

    return image;
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
    // TBD
    //setImage("in", tmp);
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
 
    //VLOG(4) << name << " update: " <<  out.refcount << " " << out_old.refcount;
  
    return true;
  }

  bool ImageNode::draw() 
  {
    
    // TBD if update is the only function to call getImage, then
    //  the imval will have been updated
    cv::Mat tmp = getImage("out");
    if (!tmp.empty()) {

      cv::Size sz = cv::Size(Config::inst()->thumb_width, Config::inst()->thumb_height);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC3);
      //cv::resize(tmp->get(), thumbnail, thumbnail.size(), 0, 0, cv::INTER_NEAREST );
      cv::resize(tmp, thumbnail, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );
       
      int fr = 1;
      if (!isDirty(this,2)) fr = 5;
      cv::Scalar col = cv::Scalar(vcol/fr);

      cv::rectangle(graph, loc + cv::Point(100,0) - cv::Point(2,2), 
          loc + cv::Point(100,0) + cv::Point(sz.width,sz.height) + cv::Point(2,2), 
          col, CV_FILLED );

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
        cv::Mat graph_roi = graph(cv::Rect(loc.x + 100, loc.y, sz.width, sz.height));
        graph_roi = cv::Scalar(0, 0, 255);
        thumbnail.copyTo(graph_roi);
      }
    }
    
    bool rv = Node::draw();

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

    cv::Mat out = getImage("out");
    if (out.empty()) return false;

    cv::imwrite(file_name.str(), out);
    write_count++;
    setSignal("write_count", write_count);

    LOG(INFO) << name << " wrote " << CLTXT << file_name << CLNRM;
    // TBD register that these frames have been saved somewhere so it is easy to load
    // them up again?
  }

  bool ImageNode::handleKey(int key)
  {
    bool valid_key = Node::handleKey(key);
    if (valid_key) return true;
    
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
    vcol = cv::Scalar(0,255,255);

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

    if (key == '.') {
      value += abs(getSignal("step"));   
    }
    else if (key == '/') {
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
    if (value > max) {
      value = max;
      step = -abs(step);   
      setSignal("step", step);
    }
    if (value < min) {
      value = min;
      step = abs(step);   
      setSignal("step", step);
    }
    
    setSignal("value", value);

    VLOG(2) << "Signal " << name << " " << value;
    //is_dirty = true;
    setDirty();

    return true;
  }

  bool Signal::draw()
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
    
    return Node::draw();
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

    // not really an input, but using inputs since outputs aren't distinct
    setInputPort(BUFFER, "out", NULL, "");

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
    
    setSignal("cur_size", frames.size());

    if (frames.size() <= 0) return false;
   
    const int ind =  ((int)getSignal("ind"))%frames.size();
    setSignal("ind", ind);

    cv::Mat out = frames[ind];
    setImage("out", out);


    if (VLOG_IS_ON(5)) {
      VLOG(15) << frames.size();
      imshow(name, out);
    }

    return true;
  }

  bool Buffer::draw() 
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

      cv::Size sz = cv::Size(Config::inst()->thumb_width * 0.25, Config::inst()->thumb_height * 0.25);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC3);
      cv::resize(frames[ind], thumbnail, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );
      cv::Mat graph_roi = graph(
          cv::Rect(loc.x + 100 + i * sz.width, loc.y + sz.height*4, sz.width, sz.height));
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
    
    return ImageNode::draw();
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
      cv::Mat tmp = cv::Mat( 
        Config::inst()->thumb_width, Config::inst()->thumb_height, CV_8UC3);
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
  }

  bool Buffer::save(cv::FileStorage& fs) 
  {
    ImageNode::save(fs);

    //fs << "max_size" << max_size;
  }
  //////////////////////////////////////////////////////////////////////////////////////////

}  // namespace bm

