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
#include "config.h"


using namespace cv;
using namespace std;

namespace bm {

  unsigned long
    hashdjb2(const char *str)
    {
      unsigned long hash = 5381;
      int c;

      while (c = (unsigned char)(*str++))
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

      return hash;
    }

  cv::Scalar hashStringColor(const string str) {
    
    unsigned long val = hashdjb2(str.c_str());

    int val2 = val % 0xffffff;
    
    int r = (val2 & 0xff0000) >> 16;
    int g = (val2 & 0x00ff00) >> 8;
    int b = (val2 & 0x0000ff) >> 0;
   
    VLOG(6) << "hash color 0x" << std::hex << val << " 0x" << std::hex << val2 
      << " : 0x" << std::hex <<  r << " 0x" << std::hex << g << " 0x" << std::hex << b;
    return cv::Scalar(r,g,b);
  }

  //////////////////////////////////////////////////////////////////////////////////////

  //Elem::Elem() : name("undefined"), highlight(false), highlight2(false)
  //{
  //}

  Elem::Elem(const string name) : name(name), highlight(false), highlight2(false)
  {
  }

  bool Elem::isDirty(const void* caller, const int ind, const bool clear) 
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

  bool Elem::setDirty()
  {
    for (map<const void*, map<int, bool> >::iterator it = dirty_hash.begin(); it != dirty_hash.end(); it++) {
      for (map<int,bool>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++) {
        it2->second = true;
      }
    }

  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Connector::Connector(const std::string name) : 
    Elem(name),
    parent(NULL),
    src(NULL),
    dst(NULL),
    //writeable(true),
    type(SIGNAL),
    saturate(0),
    val_min(0.0),
    val_max(1.0),
    value(0),
    internally_set(false)
  {

  }

  bool Connector::update()
  {
    // TBD do_update

    if (src && src->parent) {
      src->parent->update();
      //if (src->parent->isDirty(this, 0)) {
      // TBD
      if (src->isDirty(this, 0)) { //|| ((type==SIGNAL) && (value != src->value)) {
      //if (value != src->value) { // TBD could use this with images if image updates made the value change

        // now get dirtied data
        value = src->value;

        im = src->im;
       
        // if a connector has a src then it can't be an output
        // (at least until it gets overridden as an output)
        internally_set = false;
        // TBD get copy of sigbuf
        // sigbuf = src->sigbuf;
        
        setDirty();
      }
    } 

    if (saturate == ROLL) {
      //const float span = val_max - val_min;
      if (value < val_min) value = val_max; 
      if (value > val_max) value = val_min;
    } else if (saturate == SATURATE) {
      if (value < val_min) value = val_min; 
      if (value > val_max) value = val_max;
    } // saturate

  }
  
  std::string typeToString(const conType type)
  {
    if (type == IMAGE) {
      return "Image";
    } else if (type == SIGNAL) {
      return "Signal";
    } else if (type == BUFFER) {
      return "Buffer";
    } else if (type == SIGBUF) {
      return "SigBuf";
    }
    return "Unknown";
  }

  bool Connector::setImage(cv::Mat im)
  {
    if (im.empty()) return false;

    boost::mutex::scoped_lock l(im_mutex);
    this->im = im;
    setDirty();

    return true;
  }

  void Connector::draw(cv::Mat graph_ui, cv::Point2f ui_offset) 
  {
   
    cv::Scalar hash_col = hashStringColor(/*parent->name +*/ typeToString(type) + name);
    VLOG(6) << name << " " << hash_col[0] << " " << hash_col[1] << " " << hash_col[2];

    bool is_dirty = isDirty(this, 0);

    const int bval = 150 + is_dirty * 105;
    hash_col *= (is_dirty ? 1.0 :0.6);
    // draw a box around the port
    cv::Scalar rect_col = cv::Scalar(40, 50, 40);
    if (highlight) {
      rect_col = cv::Scalar(140, 80, 80);
    }
    cv::rectangle(graph_ui, 
          parent->loc + loc + ui_offset, 
          parent->loc + loc + ui_offset + cv::Point2f(name.size()*10, -10), 
          rect_col,
          CV_FILLED);

    cv::rectangle(graph_ui, 
          parent->loc + loc + ui_offset, 
          parent->loc + loc + ui_offset + cv::Point2f(name.size()*10, -10), 
          hash_col);
    
    if (internally_set) {
      cv::rectangle(graph_ui,
          parent->loc + loc + ui_offset,
          parent->loc + loc + ui_offset - cv::Point2f(5, 5),
          cv::Scalar(255, 255, 255),
          CV_FILLED);
    }

    // draw connecting line if there is a connector connected to this one
    if (src) {

      cv::Scalar dst_hash_col = hashStringColor(/*src->parent->name +*/ typeToString(src->type) + src->name);

      vector<cv::Point2f> control_points;
      control_points.resize(4);
      control_points[0] = src->parent->loc + src->loc + cv::Point2f(src->name.size()*10, -5);
      control_points[3] = parent->loc + loc + cv::Point2f(0,-5);
      

      // TBD if control_points dirty
      {
        cv::Point2f diff = control_points[3] - control_points[0];
        float dist = abs(diff.x) + abs(diff.y);

        // don't want lines going
        float y_off = 0;
        if ((control_points[3].x < control_points[0].x) && 
            (abs(control_points[3].y - control_points[0].y) < 100)) y_off = 100;

        control_points[1] = control_points[0] + cv::Point2f(dist/3.0,  y_off);
        control_points[2] = control_points[3] - cv::Point2f(dist/3.0, -y_off);
        getBezier(control_points, connector_points, 32);
      }

      // draw dark outline around curve
      for (int i = 1; i < connector_points.size(); i++) {
        cv::Scalar outline_col = cv::Scalar(10,10,10);
        if (highlight2) outline_col *= 4;
        cv::line(graph_ui, 
            connector_points[i-1] + ui_offset, 
            connector_points[i] + ui_offset, 
            outline_col, 4, CV_AA ); 
      }
      // now draw a colored interior curve
      for (int i = 1; i < connector_points.size(); i++) {
        cv::Scalar col;
        const float fr = (float)i/(float)connector_points.size();

        // colorize the lines so that starts and ends are distinct, and 
        // use the hash colors so similarly named connector ends are similarly colored.
        float wt1, wt2;
        if (fr < 0.5) {
          wt1 = 1.0 * (1.0-fr*fr);
          wt2 = 0.0;
        } else {
          wt1 = ((1.0-fr)*(1.0-fr));
          wt2 = 1.0 * (fr);
        }
        cv::Scalar hc2 = dst_hash_col * cv::Scalar(wt1);
        //cv::Scalar hc1 = hash_col *( 1.0-(1.0-fr));
        cv::Scalar hc1 = hash_col * cv::Scalar(wt2);
        col = hc1 + hc2;
        
        if (highlight2) col *= 3;
        if (src && src->highlight2) col *= 2;

        cv::line(graph_ui, 
          connector_points[i-1] + ui_offset, 
          connector_points[i] + ui_offset, 
          col, 2, CV_AA ); 
      }
    }

    stringstream port_info;
    port_info << name;

    // color based on type late
    cv::Scalar col = cv::Scalar(200,200,200);
    const int cval = 75;

    if (type == SIGNAL) {
      col = cv::Scalar(cval, bval, bval);
      port_info << " " << setprecision(4) << value;
    } else if (type == IMAGE) {
      col = cv::Scalar(bval, cval+20, bval);
    } else if (type == BUFFER) {
      col = cv::Scalar(bval, bval, cval);
    } else if (type == SIGBUF) {
      col = cv::Scalar(bval, bval/2 + cval/2, bval/2 + cval/2);
    }
   
    port_info << " " << description;

    cv::putText(graph_ui, port_info.str(), parent->loc + loc + ui_offset, 1, 1, col, 1);
  }

  // Buffers are different than Signals and Images because it isn't desirable to copy them (though maybe the overhead isn't that bad for small buffers?)
  // especially the image types.  Signal buffers may need different treatment, or some master templating scheme
  // needs to take care of it.
  #if 0
  Buffer* Connector::getBuffer()
  {
    Buffer* im_in = NULL;
    
    if (
   
    //dynamic_cast<Buffer*> (con->src->parent);
  
  }
  #endif
  
  //////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////
  Node::Node(const string name) : 
    Elem(name), 
    selected_type(NONE),
    selected_port_ind(-1),
    selected_port(""),
    do_update(false)
  {
    vcol = cv::Scalar(200,200,200);

    setSignal("enable", 1.0, false, SATURATE, 0,1);
    setSignal("force_update", 0.0);
  }
  
/*
  Node::Node(string name, cv::Point loc, cv::Mat graph_ui ) : 
    selected_type(NONE),
    selected_port_ind(-1),
    name(name),
    loc(loc),
    graph_ui(graph_ui),
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

    //boost::mutex::scoped_lock l(port_mutex);
    for (int i = 0; i < ports.size(); i++) {
      // TBD look for per-port enable here
      if (ports[i]->src) ports[i]->src->parent->setUpdate();
    }
    
    return true;
  }


  // the rv is so that an inheriting function will know whether to 
  // process or not
  bool Node::update() 
  {
    if (!do_update) return false;
    do_update = false; 

    //boost::mutex::scoped_lock l(port_mutex);
    // need to update enable no matter if it is false
    for (int i = 0; i < ports.size(); i++)
    {
      if (ports[i]->name == "enable")
        ports[i]->update();
    }

    // TBD should this occur before updating the inputs?
    if (!(getSignal("enable") >= 1.0)) 
      return false;
    
    bool inputs_dirty = false;

    for (int i = 0; i < ports.size(); i++)
    {
      // already updated if named enable
      if ((ports[i]->name != "enable") ) { // TBD provide flag to disable updating
        ports[i]->update();
      }
      // TBD may want to track per output dirtiness later?
       
      if (!ports[i]->internally_set) {
        const bool is_dirty = ports[i]->isDirty(this, 1);
        VLOG(2) << name << " " << CLTXT << ports[i]->name << " " << CLNRM << is_dirty;
        if (is_dirty) {
          inputs_dirty = true;
        }
      }
    }

    // the inheriting object needs to set is_dirty as appropriate if it
    // isn't sufficient to have inputs determine it(e.g. it is sourcing change)
    if ( inputs_dirty ) {
      setDirty();
    }

    // TBD loop through ports and run getImage on each
    // or could that be combined with looping through the getInputVector above?
    
    VLOG(1) << name << " in sz " << ports.size() << " inputs dirty " << inputs_dirty;

    return true;
  }

  bool Node::posUpdate() 
  {
    // pos update
    vel += acc;
    acc = cv::Point2f(0,0);
    loc += vel;
    vel *= 0.9;

    #if 0
    if (loc.x < 0) {
      loc.x = 0;
      vel.x = abs(vel.x)*0.5;
    }

    if (loc.x > graph_ui.cols) {
      loc.x = graph_ui.cols;
      vel.x = -abs(vel.x)*0.5;
    }
    if (loc.y < 0) {
      loc.y = 0;
      vel.y = abs(vel.y)*0.5;
    }
    if (loc.y > graph_ui.rows) {
      loc.y = graph_ui.rows;
      vel.y = -abs(vel.y)*0.5;
    }
    #endif
  }

  bool Node::draw(cv::Point2f ui_offset) 
  {
    posUpdate();
  
    if (graph_ui.empty()) {  
      LOG(ERROR) << "graph empty";
      return false;
    }
    
    int fr = 1;
    if (!isDirty(this,1)) fr = 5;
    cv::Scalar col = cv::Scalar(vcol/fr);

    if (!(getSignal("enable") >= 1.0)) 
      cv::circle(graph_ui, loc + ui_offset, 10, cv::Scalar(0,0,100), -1);

    cv::circle(graph_ui, loc + ui_offset, 24, col, 4);

    const int ht = 10;

    int j = 0;
 
    if (highlight) {
        cv::Scalar selected_color = cv::Scalar(0,220,1);
        // draw green circle on selected node
        #if 0
        cv::circle(graph_ui, 
            loc + ui_offset, 
            18, 
            selected_color*0.8, 
            -1);
        #endif

        cv::rectangle(graph_ui,
            loc + cv::Point2f(0, -10) + ui_offset, 
            loc + cv::Point2f(135, ports.size()*10 - 3) + ui_offset, 
            selected_color,
            3);
    } 

    {
      boost::mutex::scoped_lock l(port_mutex);
      // draw rectangle around entire node
      cv::rectangle(graph_ui, 
          loc + cv::Point2f(-5, -15) + ui_offset, 
          loc + cv::Point2f(140, ports.size()*10 + 2) + ui_offset, 
          vcol*0.2, //cv::Scalar(255,0,0),
          2);
    
    int max_width = 0;
    for (int i = 0; i < ports.size(); i++) {
      if (i == selected_port_ind) {
        ports[i]->highlight = true;
        if (highlight) { 
          ports[i]->highlight2 = true;
        } else {
          ports[i]->highlight2 = false;
        }
      } else {
        ports[i]->highlight = false;
        ports[i]->highlight2 = false;
      }
      ports[i]->draw(graph_ui, ui_offset);
    }
    }

    cv::putText(graph_ui, name, loc - cv::Point2f(9,  ht) + ui_offset, 1, 1, cv::Scalar(115,115,115));
    cv::putText(graph_ui, name, loc - cv::Point2f(10, ht) + ui_offset, 1, 1, cv::Scalar(255,255,255));

    return true;
  }

  bool Node::load(cv::FileNodeIterator nd)
  {
    // TBD name, loc?

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
    //fs << "vcol" << p->vcol  ; 
    
  }
  
  bool Node::handleKey(int key)
  {
    bool valid_key = true;

    VLOG(2) << selected_type << " \"" << selected_port << "\"";
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

    
      const float acc_step = 3.0;
      // TBD alternatively could handle loc as a Signal
      
            if (key == '8') {  // UP
        acc.y -= acc_step;
    //    LOG(INFO) << "acc.y " << acc.y;
      } else if (key == '2') {  // DOWN
        acc.y += acc_step;
      } else if (key == '4') {  // LEFT
        acc.x -= acc_step;
      } else if (key == '6') {  // RIGHT
        acc.x += acc_step;
      }
      else {
        valid_key = false;
      }


    return valid_key;
  }

  //////////////////////////////////////////////////////

  int Node::getIndFromPointer(Connector* con)
  {
    if (con == NULL) return -1;
    
    boost::mutex::scoped_lock l(port_mutex);
    for (int i = 0; i < ports.size(); i++) {
      if (con == ports[i]) {
        VLOG(3) << con->name << " " << i; 
        return i; 
      }
    }

    return -1;
  }
  // TBD selectPortByPointer

  bool Node::selectPortByInd(const int ind)
  {
    if (ind < 0) return false;
    boost::mutex::scoped_lock l(port_mutex);
    if (ind >= ports.size())  return false;

    selected_port_ind = ind;
    selected_port = ports[ind]->name;
    selected_type = ports[ind]->type;
    
    return true;
  }

  bool Node::getPrevPort(const conType type)
  {
    boost::mutex::scoped_lock l(port_mutex);
    for (int i = ports.size()-1; i >= 0; i--) {
      int ind = (i + selected_port_ind) % ports.size();
      
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

  bool Node::getNextPort(const conType type)
  {
    boost::mutex::scoped_lock l(port_mutex);
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
  /*
    get a pointer to the port connected to this nodes port
  */
  bool Node::getInputPort(
      const conType type, 
      const string port,
      Connector*& con,
      string& src_port)
  {
    con = NULL;
  
    boost::mutex::scoped_lock l(port_mutex);

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

  /*
    Connect a source connector port to this port

    if src_node is NULL then any existing connecting port is disconnected.
  */
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
      // TBD may not always want to create the connector if it doesn't exist
      con = new Connector(port);
      con->parent = this;
      con->type = type;
    
      boost::mutex::scoped_lock l(port_mutex);
      con->loc = cv::Point2f(0, ports.size()*10);

      ports.push_back(con);

      VLOG(2) << "\"" << con->parent->name << "\"" <<CLTX2 << " new connector " << CLNRM 
          << type << " \"" << con->name << "\"";
    }
      
    // blank out existing dst connection if it exists, it will be overwritten next
    if (con->src) {
      con->src->dst = NULL;
    }

    Connector* src_con = NULL;
    if (src_node) {
      bool src_con_exists = src_node->getInputPort(type, src_port, src_con, existing_port);
      if (!src_con_exists) {
        // this will produce connectors in the src parent in an order determined
        // by the way the earlier connections use them as ports
        src_con = new Connector(src_port);
        src_con->parent = src_node;
        src_con->type = type;
        boost::mutex::scoped_lock l(src_node->port_mutex);
        src_con->loc = cv::Point2f(0, src_node->ports.size()*10);
        
        src_node->ports.push_back(src_con);
        VLOG(2) << con->parent->name << " new src Connector " << type << " " << src_con->name; 
      }
      
      src_con->dst = con;
      VLOG(1) << "\"" << name << "\" setInputPort: " << type << " " << port << " from "
        << CLTXT /*<< inputs[type][port].first->name << " "*/  // not necessarily non-NULL
        << "\"" << src_port << "\"" << CLNRM; 
    } else {

      VLOG(1) << "\"" << name << "\" setInputPort: " << type << " " << port;
    }

    con->src = src_con;
  }
  
  bool Node::setImage(const std::string port, cv::Mat& im, const bool internally_set)
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
      con->internally_set = internally_set;
    }
     
    if (!con) {
      LOG(ERROR) << "no connector";
      return false;
    }

    // can't set signal if it is controlled by src port 
    if (con->src) return false;

    con->setImage(im);
  
    return true;
  }
 
  bool Node::setSignal(
      const std::string port, 
      const float val,
      const bool internally_set,
      const int saturate,
      const float min,
      const float max
      )
  {
    
    Connector* con = NULL;
    string src_port;
    if (!getInputPort(SIGNAL, port, con, src_port)) {
      
      // create it if it doesn't exist
      setInputPort(SIGNAL, port);
      
      if (!getInputPort(SIGNAL, port, con, src_port)) {
        LOG(ERROR) << name << " still can't get connector " << CLTXT << port << CLNRM; 
        return false;
      }
    
      // only use this setting if the port is new (TBD)
      con->internally_set = internally_set;
    }
    
    // can't set signal if it is controlled by src port 
    if (con->src) return false;

    const float val_orig = con->value;
    if (val != val_orig) {
      con->value = val;
      con->setDirty();
    }
   

    if (saturate > 0) {
      LOG(INFO) << name << " - " << port << ((saturate == 1) ? " - saturating " : " - rolling over") << min << " " << max;
      con->saturate = saturate;
      con->val_min = min;
      con->val_max = max;
    }

    return true;
  }
 
 // get an imagenode image from this nodes imagenode inputs
  cv::Mat Node::getImage(
    const string port,
    bool& valid,
    bool& is_dirty)
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
     
    {
      boost::mutex::scoped_lock l(con->im_mutex);
      im = con->im;
      is_dirty = con->isDirty(this, 2);
    }
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
    const float val,
    int& actual_ind)
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

    //Buffer* im_in = con->getBuffer(); // dynamic_cast<Buffer*> (con->src->parent);
    Buffer* im_in = dynamic_cast<Buffer*> (con->src->parent);

    if (!im_in) {
      LOG(ERROR) << name  << " " << port << " improper Buffer connected";
      return tmp;
    }

    cv::Mat image = im_in->get(val, actual_ind);
    con->im = image;

    return image;
  }

  cv::Mat Node::getBuffer(
    const std::string port,
    const int val,
    int& actual_ind
    )
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

    cv::Mat image = im_in->get(val,actual_ind);
    con->im = image;

    return image;
  }

  #if 0
  Node* Node::getBuffer(
    const std::string port)
  {
  }
  #endif

  // Buffers are different from Signals and Images, they don't have a lower level type like float or Mat they contain- the Buffer node is instead
  // passed around
  bool Node::setBuffer(
    const std::string port,
    const bool internally_set
  )
  {
    Connector* con = NULL;
    string src_port;
    if (!getInputPort(BUFFER, port, con, src_port)) {
      // create it since it doesn't exist
      setInputPort(BUFFER, port, NULL, "");
      // now get it again TBD actually check rv
      if (!getInputPort(BUFFER, port, con, src_port)) {
        LOG(ERROR) << "still can't get connector";
        return false;
      }
      con->internally_set = internally_set;
    }
    
    // can't set signal if it is controlled by src port 
    //if (con->src) return false;

    con->setDirty();
    return true;
  }

  bool Node::setSigBuf(
    const std::string port,
    const bool internally_set
  )
  {
    Connector* con = NULL;
    string src_port;
    if (!getInputPort(SIGBUF, port, con, src_port)) {
      // create it since it doesn't exist
      setInputPort(SIGBUF, port, NULL, "");
      // now get it again TBD actually check rv
      if (!getInputPort(SIGBUF, port, con, src_port)) {
        LOG(ERROR) << "still can't get connector";
        return false;
      }
      con->internally_set = internally_set;
    }
    
    // can't set signal if it is controlled by src port 
    //if (con->src) return false;

    con->setDirty();

    return true;
  }
  //////////////////////////////////////////////////////////////////////////////////////////
  ImageNode::ImageNode(const std::string name) : Node(name)
  {
    vcol = cv::Scalar(255,0,255);
  
    // create the entry for out
    cv::Mat out;
    setImage("out", out, true);
    // TBD image generators don't need this
    //setImage("in", tmp);
  }
   
  /// Probably don't want to call this in most inheriting functions, skip back to Node::update()
  bool ImageNode::update() 
  {
    const bool rv = Node::update();
    if (!rv) return false;

    // TBD make flag to control this, or derived passthrough class does this
    {
    // TBD any reason to call this here?
    cv::Mat in = getImage("in");
    if (in.empty()) {
      // create a default image
      cv::Mat out = getImage("out");
      if (out.empty()) {
        // TBD make Config return mat just like this
        cv::Size sz = Config::inst()->getImSize();
        out = cv::Mat(sz, MAT_FORMAT_C3, cv::Scalar(0));
        setImage("out", out);
        //setDirty();
      }
      return true; 
    }
    
    setImage("out", in);
    //setDirty();
    }
    //VLOG(4) << name << " update: " <<  out.refcount << " " << out_old.refcount;
  
    return true;
  }

  bool ImageNode::draw(cv::Point2f ui_offset) 
  {
    if (graph_ui.empty()) { 
      LOG(ERROR) << "graph_ui is empty";
      return false;
    }

    // TBD if update is the only function to call getImage, then
    //  the imval will have been updated
    // TBD may not want to call getImage from draw thread, instead have a copy of the image stored somewhere
    cv::Mat tmp = getImage("out").clone();
    if (!tmp.empty()) {

      cv::Size sz = cv::Size(Config::inst()->thumb_width, Config::inst()->thumb_height);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC4);
      //cv::resize(tmp->get(), thumbnail, thumbnail.size(), 0, 0, cv::INTER_NEAREST );
      cv::resize(tmp, thumbnail, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );
       
      int fr = 1;
      if (!isDirty(this,2)) fr = 5;
      cv::Scalar col = cv::Scalar(vcol/fr);

      cv::Point2f thumb_offset = cv::Point2f(0, -sz.height - 20);
     
      // draw thumbnail
      {
        cv::Point2f pth = loc + ui_offset + thumb_offset;
        float xth = pth.x;
        float yth = pth.y;

        if ((xth > 0) && (yth > 0) && 
            (xth + sz.width < graph_ui.cols) && (yth + sz.height < graph_ui.rows)) {
          
          cv::Point2f border_off  = cv::Point2f(2,2);
          cv::rectangle(graph_ui,
            pth - border_off, pth + border_off, col, CV_FILLED);

          cv::Mat graph_roi = graph_ui(cv::Rect(xth, yth, sz.width, sz.height));
          graph_roi = cv::Scalar(0, 0, 255);
          thumbnail.copyTo(graph_roi);
        }
      }
    }
    
    bool rv = Node::draw(ui_offset);

    return rv;
  }

  bool ImageNode::writeImage()
  {
    LOG(INFO) << name << " writing ImageNode image to disk";

    if (dir_name.str() == "") {
      time_t t1 = time(NULL);

      // TBD define path to data somewhere to be reused by all
      dir_name << "../data/record/" << t1 << "_" << name;
      LOG(INFO) << name << " creating directory" << dir_name.str();
    }
    
    boost::filesystem::create_directories(dir_name.str());

    stringstream file_name;
    int write_count = getSignal("write_count");
    file_name << dir_name.str() << "/image_" << (write_count + 1000000) << ".png";

    cv::Mat out = getImage("out").clone();
    if (out.empty()) return false;

    cv::Mat tmp0 = cv::Mat(out.size(), CV_8UC3, cv::Scalar(0));  
    // just calling reshape(4) doesn't do the channel reassignment like this does
    int ch[] = {0,0, 1,1, 2,2};
    cv::mixChannels(&out, 1, &tmp0, 1, ch, 3 );

    cv::imwrite(file_name.str(), tmp0);
    write_count++;
    setSignal("write_count", write_count);

    LOG(INFO) << name << " wrote " << CLTXT << file_name.str() << CLNRM;
    // TBD register that these frames have been saved somewhere so it is easy to load
    // them up again?
  }

  bool ImageNode::handleKey(int key)
  {
    bool valid_key = Node::handleKey(key);
    if (valid_key) return true;
   
    valid_key = true;
    if (key == 'p') {
      writeImage(); 
    } 
    else {
      valid_key = false;
    }

    return valid_key;
  }

  /**
    convenience function to get opencv interpolation mode type
    */
  int ImageNode::getModeType() // TBD supply string optionally/
  {
    const string port = "mode";
    Connector* con = NULL;
    string src_port;
    // then look at input nodes
    getInputPort(SIGNAL, port, con, src_port);


    // used to do rollover here, but now require all calling 
    // nodes to set it up properly with
    // setSignal("mode", 0, ROLL, 0, 4);
    const int mode = getSignal(port);
    int mode_type = cv::INTER_NEAREST;
    con->description = "NEAREST";
    if (mode == 1) {
      mode_type = cv::INTER_LINEAR;
      con->description = "LINEAR";
    } else if (mode == 2) { 
      mode_type = cv::INTER_AREA;
      con->description = "AREA";
    } else if (mode == 3) { 
      mode_type = cv::INTER_CUBIC;
      con->description = "CUBIC";
    } else if (mode == 4) { 
      mode_type = cv::INTER_LANCZOS4;
      con->description = "LANCZOS4";
    }
    VLOG(2) << mode << " " << con->description;
    return mode_type;
  }

  int ImageNode::getBorderType(const bool avoid_wrap)
  { 
    string port = "border";
    int border = getSignal(port);
    
    Connector* con = NULL;
    string src_port;
    // then look at input nodes
    getInputPort(SIGNAL, port, con, src_port);

    border += 5;
    border %= 5;
    setSignal("border", border);

    int border_type = BORDER_CONSTANT;
    con->description = "CONST";
    if (border == 1) {
      border_type = BORDER_REFLECT;
      con->description = "REFLECT";
    }
    else if (border == 2 && !avoid_wrap) {
      border_type = BORDER_WRAP;
      con->description = "WRAP";
    }
    else if (border == 3) {
      border_type = BORDER_REPLICATE;
      con->description = "REPLICATE";
    }
    else if (border == 4) {
      border_type = BORDER_REFLECT_101;
      con->description = "REFLECT_101";
    }
    return border_type;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  // TBD subclasses of Node that are input/output specific, or make that general somehow?
  Signal::Signal(const std::string name) : Node(name)
  {
    vcol = cv::Scalar(0,255,255);

    setSignal("value", 0, true);
    setSignal("min", 0);
    setSignal("max", 1);
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

    if (key == '\'') {
      value = getSignal("min");   
    }
    else if (key == '\\') {
      value = getSignal("max");
    } 
    else if (key == 73) {
      value += getSignal("step");   
    }
    else if (key == 81) {
      value -= getSignal("step");
    } 
    else {
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

    //VLOG(4) << "Signal " << name << " " << value;

    return true;
  }

  bool Signal::draw(cv::Point2f ui_offset)
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

    // TBD make a graphic that shows a rolling oscilloscope value
    #if 0
    if (!graph_ui.empty()) {
    cv::rectangle(graph_ui, loc, 
        loc + cv::Point2f( x * 50.0 , 5) + ui_offset, 
        cv::Scalar(255, 255, 100), CV_FILLED);
    }
    #endif

    return Node::draw(ui_offset);
  }

  bool Signal::load(cv::FileNodeIterator nd)
  {
    Node::load(nd);
  }

  bool Signal::save(cv::FileStorage& fs) 
  {
    Node::save(fs);
  }
  
  //////////////////////////////////////////////////////////////////////////////////////////
  Buffer::Buffer(const std::string name) : ImageNode(name)
  {
    //this->max_size = max_size;
    //LOG(INFO) << "new buffer max_size " << this->max_size;
    vcol = cv::Scalar(200, 30, 200);

    cv::Mat tmp;
    setImage("in", tmp);
    // not really an input, but using inputs since outputs aren't distinct
    setBuffer("out", true);
    //setInputPort(BUFFER, "out", NULL, "");

    //setImage("image", cv::Mat());
    setSignal("max_size", 100);
  }
 
  bool Buffer::manualUpdate()
  {
    bool b1, con_is_dirty;
    cv::Mat in = getImage("in", b1, con_is_dirty);
    if (!in.empty() && con_is_dirty) 
      add(in); 
   
    const int cur_size =  frames.size();
    setSignal("cur_size", cur_size, true);
    if (cur_size <= 0) return false;
    //const int ind =  ((int)getSignal("ind") + cur_size) % cur_size;
    //setSignal("ind", ind);

    return true;
  }

  bool Buffer::setOut()
  {
    boost::mutex::scoped_lock l(frames_mutex);
    if (frames.size() <= 0) return false;
    int ind = (int)getSignal("ind");
    if (ind < 0) ind = 0;
    if (ind >= frames.size()) ind = frames.size()-1;
    cv::Mat out = frames[ind];
    setImage("out", out);
    return true;
  }

  bool Buffer::update()
  {
    //VLOG(1) << name << " buffer update";
    bool rv = Node::update(); // ImageNode::update();
    if (!rv) return false;
  
    // don't want to buffer identical images
    const bool rv1 = isDirty(this,21);
    if (!rv1) { return true;}
  
    /*
    float val = getSignal("max_size");
    max_size = val;
    if (max_size < 1) max_size = 1;
    */
    if (!manualUpdate()) return true;

        // TBD may not always want to do this
    setOut();

    // clear any dirtiness derived from setting the image or cur ind etc.
    const bool rv2 = isDirty(this,21);
    // TBD don't have to do that anymore, out dirtiness is overlooked?
  
    return true;
  }

  bool Buffer::draw(cv::Point2f ui_offset) 
  {
    cv::Mat out = getImage("out").clone();
    // draw some grabs of the beginning frame, and other partway through the buffer 
    boost::mutex::scoped_lock l(frames_mutex);
    for (int i = 1; i < 4; i++) {
      int ind = i * frames.size() / 3;
      if (i == 3) ind = frames.size() - 1;
      if (ind >= frames.size()) continue;
      if (ind < 0) continue;
      
      cv::Mat frame = frames[ind].clone(); // seems to be a segfault related to this

      if (frame.empty()) { 
        VLOG(2) << "frames " << i << CLERR << " is empty" << CLNRM;  continue; 
      }

      // TBD make this optional
      if (out.empty()) out = frames[0];//.clone();

      cv::Size sz = cv::Size(Config::inst()->thumb_width * 0.25, Config::inst()->thumb_height * 0.25);

      cv::Mat thumbnail = cv::Mat(sz, CV_8UC4);
      cv::resize(frame, thumbnail, sz, 0, 0, cv::INTER_NEAREST );
      //cv::resize(tmp->get(), thumbnail, cv::INTER_NEAREST );

      const float xth = loc.x + ui_offset.x + 100 + i * sz.width;
      const float yth = loc.y + ui_offset.y + sz.height*4;

      if ((xth > 0) && (yth > 0) && 
          (xth + sz.width < graph_ui.cols) && (yth + sz.height < graph_ui.rows)) {

        cv::Mat graph_roi = graph_ui(cv::Rect(xth, yth, sz.width, sz.height));
        graph_roi = cv::Scalar(0, 0, 255);
        thumbnail.copyTo(graph_roi);
      }
    }
   
    int max_size = getSignal("max_size");
    if (max_size < 1) {
      max_size = 1;
      setSignal("max_size", max_size);
    }
  
    if (frames.size() < getSignal("max_size"))
      vcol = cv::Scalar(200, 30, 200);
    else
      vcol = cv::Scalar(55, 255, 90);
    
    return ImageNode::draw(ui_offset);
  }
  
  bool Buffer::addCore(cv::Mat& new_frame, bool restrict_size)
  {
    if ((frames.size() > 0) && 
        (new_frame.refcount == frames[frames.size()-1].refcount)) {
      new_frame = new_frame.clone();
      LOG_FIRST_N(INFO,15) << name << " cloning identical frame " 
          << new_frame.refcount << " " << frames[frames.size()-1].refcount;
      //return false;
    }

    frames.push_back(new_frame);
    
    if (restrict_size) {
      while (frames.size() > getSignal("max_size")) frames.pop_front();
    }

    VLOG(4) << name << " sz " << frames.size();
    return true;
  }

  bool Buffer::add(cv::Mat& new_frame, bool restrict_size)
  {
    if (new_frame.empty()) {
      LOG(ERROR) << name << CLERR << " new_frame is empty" << CLNRM;
      return false;// TBD LOG(ERROR)
    }
   
    {
      boost::mutex::scoped_lock l(frames_mutex);
      addCore(new_frame, restrict_size);
    }

    {
      // TBD is this really necessary?
      Connector* con;
      string src_port;
      if (getInputPort(BUFFER, "out", con, src_port)) {
        con->setDirty();
      }
    }
   
    // TBD is_dirty wouldn't be true for callers that want frames indexed from beginning if no pop_front has been done.
    setDirty();

    return true;
  }
  
  cv::Mat Buffer::get() {
    return get(0);
  }

  // not the same as the inherited get on purpose
  // many callers per time step could be calling this
  cv::Mat Buffer::get(const float fr, int& actual_ind) 
  {
    const int ind = (int)(fr * (float)frames.size());
    actual_ind = ind;
    VLOG(6) << ind << " " << fr << " " << frames.size();
    //if (fr < 0) {
    //  ind = frames.size() - ind;
    //}
    
    return get(ind);
  }

  cv::Mat Buffer::get(int ind, int& actual_ind)
  {
    boost::mutex::scoped_lock l(frames_mutex);
    if (frames.size() < 1) {
      VLOG(1) << "no frames returning gray";
      cv::Mat tmp = cv::Mat( 
        Config::inst()->thumb_width, Config::inst()->thumb_height, CV_8UC4);
      tmp = cv::Scalar(128);
      return tmp;
    }
    //if (ind > frames.size() - 1) ind = frames.size() - 1;
    //if (ind < 0) ind = 0;
    ind %= frames.size();
    actual_ind = ind;
  
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
    dir_name << "../data/record/" << t1 << "_" << name;
   
    boost::mutex::scoped_lock l(frames_mutex);
    if (frames.size() == 0) return false;

    boost::filesystem::create_directories(dir_name.str());
    LOG(INFO) << name << " writing " << frames.size() 
        << " Buffer images to disk " << CLTXT << dir_name.str() << CLNRM;

    // TBD move to another thread
    for (int i = 0; i < frames.size(); i++) {
      stringstream file_name;
      file_name << dir_name.str() << "/buffer_" << (i + 1000000) << ".jpg";
      cv::imwrite(file_name.str(), frames[i]);
      VLOG(1) << file_name.str();  
      int write_count = getSignal("write_count");
      write_count++;
      setSignal("write_count", write_count);
    }
    // TBD register that these frames have been saved somewhere so it is easy to load
    // them up again?
    return true;
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
  Mux::Mux(const std::string name) : Buffer(name)
  {
    //this->max_size = max_size;
    //LOG(INFO) << "new buffer max_size " << this->max_size;
    vcol = cv::Scalar(200, 30, 200);

    // not really an input, but using inputs since outputs aren't distinct

    setSignal("cur_size", 2, true);
    cv::Mat tmp;
    cv::Size sz = Config::inst()->getImSize();
    tmp = cv::Mat(sz, MAT_FORMAT_C3, cv::Scalar(0));
    setImage("inp0", tmp);
    setImage("inp1", tmp);
    frames.resize(2);
    frames[0] = tmp;
    frames[1] = tmp;
  }
 

  bool Mux::update()
  {
    bool rv = Node::update(); // ImageNode::update();
    if (!rv) return false;
  
    // don't want to buffer identical images
    if (!isDirty(this,21)) { return true;}
 
    // first pass to determine size
    int cur_size = 0;
    for (int i = 0; i < ports.size(); i++) {
      if (ports[i]->type != IMAGE) continue;
      const string port = ports[i]->name;
      if (port.substr(0,3) != "inp") {
        VLOG(5) << name << " : " << port.substr(0,3) << " " << port;
        continue;
      }
      cur_size++;
    }
  
    {
    boost::mutex::scoped_lock l(frames_mutex);
    frames.resize(cur_size);
    setSignal("cur_size", cur_size);
   
    // second pass to copy Mat references into frames
    int ind = 0;
    for (int i = 0; (i < ports.size()) && (ind < frames.size()); i++) {
      if (ports[i]->type != IMAGE) continue;
      const string port = ports[i]->name;
      if (port.substr(0,3) != "inp") {
        VLOG(5) << name << " : " << port.substr(0,3) << " " << port;
        continue;
      }
      
      VLOG(4) << "port " << ind << " " << port;
      frames[ind] = getImage(port);
      ind++;
    }
    }
    
    setOut();

    return true;
  }

  bool Mux::handleKey(int key)
  {
    bool valid_key = Buffer::handleKey(key);
    if (valid_key) return true;
   
    valid_key = true;
    if (key == '[') {
    
      // add an input addition port, TBD move to function
      int add_num = 0;
      for (int i = 0; i < ports.size(); i++) {
        if (ports[i]->type != IMAGE) continue;
        const string port = ports[i]->name;
        
        if (port.substr(0,3) != "inp") {
          VLOG(1) << name << " : " << port.substr(0,2) << " " << port;
          continue;
        }
        add_num++;
      }
      setSignal("cur_size", add_num+1);

      // add a new addition port
      const string port = "inp" + boost::lexical_cast<string>(add_num);
      setInputPort(IMAGE, port, NULL, "out");
       
      // TBD make a way to delete a port
    } else {
      valid_key = false;
    }

    // TBD 
    if (valid_key) setDirty();
    
    return valid_key;
  }


  ////////////////////////
  // Muxes of Buffers are interesting since there is no Buffer of Buffers object and associated Taps,
  // this makes a connection so any Tap operating on a Buffer coming through the out of this node will get forwarded to
  // the proper source Buffer determined by ind.  A properly templated Buffer type might be able to make this simpler and more 
  // inuitive.
  MuxBuffer::MuxBuffer(const std::string name) :
    Buffer(name), 
    selected_buffer(NULL) 
  {
    vcol = cv::Scalar(200, 30, 200);

    setSignal("ind",0);
    setSignal("cur_size", 2);
    setBuffer("out");
    setBuffer("inp0");
    setBuffer("inp1");
  }
 

  bool MuxBuffer::update()
  {
    //VLOG(1) << name << "mux buffer update";
    bool rv = Node::update(); // ImageNode::update();
    if (!rv) return false;
  
    // don't want to buffer identical images
    if (!isDirty(this, 21)) { return true; } 

    int cur_size = getSignal("cur_size");
    const int ind = ((int)getSignal("ind")) % cur_size;

    // first pass to determine size
    cur_size = 0;
    for (int i = 0; i < ports.size(); i++) {
      if (ports[i]->type != BUFFER) continue;
      const string port = ports[i]->name;
      if (port.substr(0,3) != "inp") {
        VLOG(5) << name << " : " << port.substr(0,3) << " " << port;
        continue;
      }
      if (cur_size == ind) {
        if ((ports[i]->src) && (ports[i]->src->parent)) {
          selected_buffer = dynamic_cast<Buffer*> (ports[i]->src->parent);
        }
      }
      cur_size++;
    }
    boost::mutex::scoped_lock l(frames_mutex);
    frames.resize(cur_size);
    setSignal("cur_size", cur_size);
  
    #if 0
    // second pass to copy Mat references into frames
    int ind = 0;
    for (int i = 0; (i < ports.size()) && (ind < frames.size()); i++) {
      if (ports[i]->type != BUFFER) continue;
      const string port = ports[i]->name;
      if (port.substr(0,3) != "inp") {
        VLOG(5) << name << " : " << port.substr(0,3) << " " << port;
        continue;
      }

      frames[ind] = getImage(port);
      ind++;
    }
    #endif

    return true;
  }

  // not the same as the inherited get on purpose
  // many callers per time step could be calling this
  cv::Mat MuxBuffer::get(const float fr, int& actual_ind) 
  {
    cv::Mat tmp;
    if (!selected_buffer) {
      return tmp;
    }
    return selected_buffer->get(fr, actual_ind);
  }

  cv::Mat MuxBuffer::get(int ind, int& actual_ind)
  {
    cv::Mat tmp;
    if (!selected_buffer) return tmp;
    return selected_buffer->get(ind, actual_ind);
  }

  bool MuxBuffer::handleKey(int key)
  {
    bool valid_key = Buffer::handleKey(key);
    if (valid_key) return true;
   
    valid_key = true;
    if (key == '[') {
    
      // add an input addition port, TBD move to function
      int add_num = 0;
      for (int i = 0; i < ports.size(); i++) {
        if (ports[i]->type != IMAGE) continue;
        const string port = ports[i]->name;
        
        if (port.substr(0,3) != "inp") {
          VLOG(1) << name << " : " << port.substr(0,2) << " " << port;
          continue;
        }
        add_num++;
      }
      setSignal("cur_size", add_num+1);

      // add a new addition port
      const string port = "inp" + boost::lexical_cast<string>(add_num);
      setInputPort(IMAGE, port, NULL, "out");
       
      // TBD make a way to delete a port
    } else {
      valid_key = false;
    }

    // TBD 
    if (valid_key) setDirty();
    
    return valid_key;
  }



  //////////////////////////////////////////////////////////////////////////////////////////

}  // namespace bm

