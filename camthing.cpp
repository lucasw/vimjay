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
#include <sstream>
#include <stdio.h>
#include <time.h>
#include <typeinfo>
#include <cxxabi.h> // non portable

#include <deque>
//#include <pair>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "nodes.h" 
#include "misc_nodes.h" 
#include "filter.h"
#include "generate.h"
#include "screencap.h"
#include "camthing.h"

using namespace cv;
using namespace std;

DEFINE_string(graph_file, "../graph.yml", "yaml file to load with graph in it");
DEFINE_string(config_file, "../config.yml", "configuration settings for ui and output");

//DEFINE_bool(
namespace bm {

  std::string logMat(const cv::Mat& m) 
  {
    std::stringstream s;
    s << "{";
    for (int x = 0; x < m.rows; x++) {
      for (int y = 0; y < m.cols; y++) {
        s << " " << m.at<double>(x,y); 
      }
      if (x < m.rows-1) s << std::endl;
    }
    s << " }";
    return s.str();
  }

  bool getBezier(
      const std::vector<cv::Point2f>& control_points, // TBD currently has to be 4
      std::vector<cv::Point2f>& output_points,
      const int num // numbe of intermediate points to generate 
      )
  {
    if (control_points.size() != 4) {
      LOG(ERROR) << control_points.size() << " != 4"; 
      return false;
    }
    // TBD how to generate programmatically
    double coeff_raw[4][4] = {
      { 1, 0, 0, 0},
      {-3, 3, 0, 0},
      { 3,-6, 3, 0},
      { 1, 3,-3, 1}
    };
    cv::Mat coeff = cv::Mat(4, 4, CV_64F, coeff_raw);

    float x0 = control_points[0].x;
    float y0 = control_points[0].y;

    cv::Mat control = cv::Mat::zeros(4, 2, CV_64F);
    for (int i = 1; i < control.rows; i++) {
      control.at<double>(i, 0) = control_points[i].x - x0;
      control.at<double>(i, 1) = control_points[i].y - y0;
    }

    VLOG(5) << CLTXT << "coeff " << CLNRM << std::endl << logMat(coeff); 
    VLOG(5) << CLTXT <<"control " << CLNRM << std::endl << logMat(control); 

    cv::Point2f old_pt;

    output_points.clear();

    for (int i = 0; i < num; i++) {
      float t = (float)i/(float)(num-1);
      double tee_raw[1][4] = {{ 1.0, t, t*t, t*t*t}};

      cv::Mat tee = cv::Mat(1, 4, CV_64F, tee_raw);

      cv::Mat pos = tee * coeff * control;

      cv::Point new_pt = cv::Point2f(pos.at<double>(0,0) + x0, pos.at<double>(0,1) + y0);

      output_points.push_back(new_pt);

      VLOG(5) << "pos " << t << " "
        << new_pt.x << " " << new_pt.y 
        << std::endl << logMat(tee) 
        << std::endl << logMat(pos); 
    }

    return true;
  }

  Config* Config::instance = NULL;

  Config* Config::inst()  
  {
    if (!instance) { 
      instance = new Config;

      instance->width = 1280;
      instance->height = 720; 
      instance->thumb_width = 40; 
      instance->thumb_height = 30;
      instance->im_width = 640;
      instance->im_height = 480;

      instance->load(FLAGS_config_file);
    }

    return instance;
  }

  bool Config::load(const string config_file)
  {
    LOG(INFO) << "loading config " << config_file;
    
    FileStorage fs; 
    fs.open(config_file, FileStorage::READ);
    
    if (!fs.isOpened()) {
      LOG(ERROR) << "couldn't open " << config_file;
      return false;
    }

    fs["width"] >> width;
    fs["height"] >> height;
    fs["thumb_width"] >> thumb_width;
    fs["thumb_height"] >> thumb_height;
    fs["im_width"] >> im_width;
    fs["im_height"] >> im_height;
    
    LOG(INFO) << width << " " << height;
    LOG(INFO) << thumb_width << " " << thumb_height;
    LOG(INFO) << im_width << " " << im_height;
  }

string getId(Node* ptr) 
  {
    int status; 
    return (abi::__cxa_demangle(typeid(*ptr).name(), 0, 0, &status));
  }

class CamThing 
{
  // TBD informal timer for the system
  
  // make sure all Nodes are stored here
  deque<Node*> all_nodes;

  // the final output 
  // TBD make this a special node type
  Output* output_node;

  public:

  // conveniently create and store node
  template <class nodeType>
    nodeType* getNode(string name = "", cv::Point loc=cv::Point(0.0, 0.0))
    {
      nodeType* node = new nodeType();//name, loc, graph_im);
      
      node->name = name;
      node->loc = loc;
      node->graph = graph_im;

      all_nodes.push_back(node);
      
      LOG(INFO) << CLVAL << all_nodes.size() - 1 << CLNRM 
          << " new node " << name << " " << loc.x << ", " << loc.y;

      return node;
    }

  // delete all the nodes
  bool clearNodes()
  {
    LOG(INFO) << "clearing nodes";
    for (int i = 0; i < all_nodes.size(); i++) {
      // TBD use smart pointers
      delete all_nodes[i];
    }

    all_nodes.resize(0);
    
    output_node = NULL;
  }

  void clearAllNodeUpdates() 
  {
    for (int i = 0; i < all_nodes.size(); i++) {
      all_nodes[i]->do_update = false;
      // TBD for asynchronous this fails, but need buffering to make that work anyhow
      //all_nodes[i]->is_dirty = false;
    }
  }

  int getIndFromPointer(Node* node)
  {
    for (int i = 0; i < all_nodes.size(); i++) {
      if (all_nodes[i] == node) return i; 
    }

    return -1;
  }
  
  

  /// save the graph to an output yaml file
  bool saveGraph(std::string graph_file="graph.yml") 
  {
    LOG(INFO) << "saving graph " << graph_file;
    cv::FileStorage fs(graph_file, cv::FileStorage::WRITE);

    // TBD save date and time
    time_t rawtime; time(&rawtime);
    fs << "date" << asctime(localtime(&rawtime)); 
    
    fs << "nodes" << "[";
    for (int i = 0; i < all_nodes.size(); i++) {

      fs << "{";
      
      fs << "ind" << i; //(int64_t)all_nodes[i];   // 
      
      all_nodes[i]->save(fs);
      
      // save inputs
      fs << "inputs" << "[";
      // TBD put in Node::save method
      for (int j = 0; j < all_nodes[i]->ports.size(); j++) {
          
          Connector* con = all_nodes[i]->ports[j];

          fs << "{";
          fs << "type" << con->type;
          fs << "name" << con->name;
          // TBD loc
          if (con->type == SIGNAL) {
            //fs << it->first << it->second;  // this is clever but I don't know how to load it
            fs << "value" << con->value;
          }

          string src_port = "";
          int ind = -1;
          if (con->src) {
            src_port = con->src->name;
            // TBD this function is currently why this isn't in the Node::save()
            ind = getIndFromPointer(con->src->parent);  
          }

          fs << "ind" << ind;
          fs << "src_port" << src_port;

          fs << "}";
      }
      fs << "]";

      fs << "}";
    }
    fs << "]";
    fs.release();
    return true;
  }

  // organize all the nodes, for now based on index but later perhaps based on connectivity
  bool gridGraph() 
  {
    const int wd = (sqrt(all_nodes.size()));
    const int ht = all_nodes.size()/wd + 0.5;
    
    float dx = graph_im.cols / (wd + 1.5);
    float dy = graph_im.rows / (ht);
    
    LOG(INFO) << "making " << all_nodes.size() << " graph items into grid " << wd << " " << dx << " " << dy;

    for (int y = 0; y <= wd; y++) {
    for (int x = 0; x <= wd; x++) {
      const int ind = y*(wd+1) + x;
      if (ind >= all_nodes.size()) continue;
      
      cv::Point loc = cv::Point( x*dx + dx/4.0, y*dy + dy/4.0 );
      
      LOG(INFO) << ind << " " << all_nodes[ind]->name << " " 
          << all_nodes[ind]->loc.x << " " << all_nodes[ind]->loc.y 
          << " -> " << loc.x << " " << loc.y ;
      
      all_nodes[ind]->loc = loc;

    }}
  }

  Webcam* cam_in;
  int count;

  cv::Mat test_im;

  cv::Mat cam_image;
  cv::Mat graph_im;

  // TBD make struct for these two
  int selected_ind;
  Node* selected_node;
  conType selected_type;
  string selected_port;
  int selected_port_ind;

  // store a node to be connected to a different selected node
  int source_ind;
  Node* source_node;
  // ImageNode, Signal, or Buffer for now- TBD use enums instead of strings
  conType source_type;
  string source_port;
  int source_port_ind;
  
  CamThing() : 
      selected_ind(0), 
      selected_node(NULL),
      selected_type(NONE),
      selected_port(""),
      selected_port_ind(0),
      source_ind(0),
      source_node(NULL),
      source_type(NONE),
      source_port(""),
      source_port_ind(0),
      output_node(NULL),
      draw_nodes(true),
      paused(true)
  {
    count = 0;

    // TBD make internal type a gflag
    // or have a config file that loads it and resolution also
    graph_im = cv::Mat(cv::Size(Config::inst()->width, Config::inst()->height), MAT_FORMAT_C3);
    graph_im = cv::Scalar(0);

    if (FLAGS_graph_file =="") {
      defaultGraph();
    } else {
      loadGraph(FLAGS_graph_file);
      saveGraph("graph_load_test.yml");
    }

    cv::namedWindow("graph_im", CV_GUI_NORMAL);
    cv::moveWindow("graph_im", 0, 500);
    cv::resizeWindow("graph_im", graph_im.size().width, graph_im.size().height);
/*
    // Bring this back when there is a fullscreen/decoration free output window
    cv::namedWindow("out", CV_GUI_NORMAL);
    cv::moveWindow("out", 420, 0);
*/
  }

  ///////////////////////////////////////////////
  bool loadGraph(const std::string graph_file)
  {
    LOG(INFO) << "loading graph " << graph_file;
    
    FileStorage fs; 
    fs.open(graph_file, FileStorage::READ);
    
    if (!fs.isOpened()) {
      LOG(ERROR) << "couldn't open " << graph_file;
      return false;
    }
  
    FileNode nd = fs["nodes"]; 
    if (nd.type() != FileNode::SEQ) {
      LOG(ERROR) << "no nodes";

      return false;
    }

    for (FileNodeIterator it = nd.begin(); it != nd.end(); ++it) {
      string type_id = (*it)["typeid"];
      string name;
      (*it)["name"] >> name;
      cv::Point loc;
      loc.x = (*it)["loc"][0];
      loc.y = (*it)["loc"][1];
      bool enable;
      (*it)["enable"] >> enable;
      
      Node* node;
      if (type_id.compare("bm::Webcam") == 0) {
        // TBD make a version of getNode that takes a type_id string
        Webcam* cam_in = getNode<Webcam>(name, loc);
        node = cam_in;

        test_im = cam_in->getImage("out").clone();
        test_im = cv::Scalar(200,200,200);

      } else if (type_id.compare("bm::ScreenCap") == 0) {
        node = getNode<ScreenCap>(name, loc);
        node->update();
      } else if (type_id.compare("bm::ImageNode") == 0) {
        node = getNode<ImageNode>(name, loc);
      } else if (type_id.compare("bm::Sobel") == 0) {
        node = getNode<Sobel>(name, loc);
      } else if (type_id.compare("bm::Buffer") == 0) {
        node = getNode<Buffer>(name, loc);
      } else if (type_id.compare("bm::ImageDir") == 0) {
        node = getNode<ImageDir>(name, loc);
      } else if (type_id.compare("bm::Add") == 0) {
        node = getNode<Add>(name, loc);
      } else if (type_id.compare("bm::Multiply") == 0) {
        node = getNode<Multiply>(name, loc);
      } else if (type_id.compare("bm::Resize") == 0) {
        node = getNode<Resize>(name, loc);
      } else if (type_id.compare("bm::Rot2D") == 0) {
        node = getNode<Rot2D>(name, loc);
      } else if (type_id.compare("bm::Signal") == 0) {
        node = getNode<Signal>(name, loc);
      } else if (type_id.compare("bm::Saw") == 0) {
        node = getNode<Saw>(name, loc);
      } else if (type_id.compare("bm::Tap") == 0) {
        node = getNode<Tap>(name, loc);
      } else if (type_id.compare("bm::TapInd") == 0) {
        node = getNode<TapInd>(name, loc);
      } else if (type_id.compare("bm::Bezier") == 0) {
        node = getNode<Bezier>(name, loc);
      } else if (type_id.compare("bm::Random") == 0) {
        node = getNode<Random>(name, loc);
      } else if (type_id.compare("bm::Output") == 0) {
        node = getNode<Output>(name, loc);
        output_node = (Output*)node;
      } else {
        LOG(WARNING) << "unknown node type " << type_id << ", assuming imageNode";
        node = getNode<ImageNode>(name, loc);
      }

      if (dynamic_cast<ImageNode*>(node)) {
        (dynamic_cast<ImageNode*> (node))->setImage("out", test_im);
      }
      node->load(it);

      if (name == "output") {
        output_node = (Output*)node;
        cv::Mat tmp;
        node->setImage("in", tmp); 
      }

      LOG(INFO) << type_id << " " << CLTXT << name << CLVAL << " " 
          << node  << " " << loc << " " << enable << CLNRM;

    }

    // second pass for inputs
    for (FileNodeIterator it = nd.begin(); it != nd.end(); ++it) {
      int ind;
      (*it)["ind"] >> ind;
      for (int i = 0; i < (*it)["inputs"].size(); i++) {
        int input_ind;
        string type;
        string port;
        string src_port;

        (*it)["inputs"][i]["type"] >> type;
        (*it)["inputs"][i]["port"] >> port;
        (*it)["inputs"][i]["ind"] >> input_ind;
        (*it)["inputs"][i]["src_port"] >> src_port;
      
        LOG(INFO) << "input " << ind << " " << all_nodes[ind]->name 
            << " " << input_ind << " " << type << " " << port << " " << input_ind
            << " " << src_port;
       
        conType con_type = NONE;
        if (type == "ImageNode") con_type = IMAGE;
        if (type == "ImageOut") con_type = IMAGE;
        if (type == "Signal") con_type = SIGNAL;
        if (type == "Buffer") con_type = BUFFER;
        // this method will produce input ports in disorder
        if (input_ind >= 0)
          all_nodes[ind]->setInputPort(con_type, port, all_nodes[input_ind], src_port);
        else 
          all_nodes[ind]->setInputPort(con_type, port, NULL, src_port);
      }
    } // second input pass

    if (output_node == NULL) {
      LOG(WARNING) << CLWRN << "No output node found, setting it to " 
          << all_nodes[all_nodes.size() - 1]->name << CLNRM;
      // TBD could make sure that this node is an output node
      
      output_node = (Output*) all_nodes[all_nodes.size() - 1];
    }

    LOG(INFO) << all_nodes.size() << " nodes total";
    //output_node->loc = cv::Point(graph.cols - (test_im.cols/2+100), 20);
    

  } // loadGraph

  void defaultGraph() 
  {
    Node* node;
    
    // create a bunch of nodes but don't connect them, user will do that
    //node = getNode<ScreenCap>(name, loc);
    //node->update();
    
    cv::Point2f loc = cv::Point2f(400,400);
    
    ImageDir* im_dir = getNode<ImageDir>("image_dir", loc);
    im_dir->dir = "../data/";
    im_dir->loadImages();

    node = getNode<Sobel>("sobel", loc);
    node = getNode<Add>("add0", loc);
    node = getNode<Add>("add1", loc);
    //node = getNode<Fir>("fir", loc);
    node = getNode<Resize>("resize", loc);
    node = getNode<Rot2D>("rot2d", loc);
    node = getNode<Signal>("signal0", loc);
    node = getNode<Signal>("signal1", loc);
    node = getNode<Saw>("saw0", loc);
    node = getNode<Saw>("saw1", loc);
    node = getNode<Saw>("saw2", loc);
    node = getNode<Tap>("tap0", loc);
    node = getNode<Tap>("tap1", loc);
    node = getNode<Tap>("tap2", loc);
    node = getNode<TapInd>("tapind0", loc);
    node = getNode<TapInd>("tapind1", loc);
    node = getNode<TapInd>("tapind2", loc);
    node = getNode<Buffer>("buffer0", loc);
    node = getNode<Buffer>("buffer1", loc);
    node = getNode<Buffer>("buffer2", loc);

    node = getNode<ImageNode>("output", loc);
    cv::Mat tmp;
    node->setImage("in", tmp); 

    //node = getNode<Output>("out", loc);

    gridGraph(); 

  #if MAKE_FIR
    {
      FilterFIR* fir = getNode<FilterFIR>("fir_filter", cv::Point(500,150));

      // http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
      static float arr[] =  { -1.0, 3.0, -3.0, 1.0, }; //{ 0.85, -0.35, 0.55, -0.05, };
       /* {
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          0.2, -0.1,
          };*/
      /*  { +0.0000000000, -0.0025983155, +0.0000000000, +0.0057162941,
          +0.0000000000, +0.0171488822, +0.0000000000, -0.1200421755,
          -0.0000000000, +0.6002108774, +1.0000000000, +0.6002108774,
          -0.0000000000, -0.1200421755, +0.0000000000, +0.0171488822,
          +0.0000000000, +0.0057162941, +0.0000000000, -0.0025983155,
          +0.0000000000,
        };*/

      vector<float> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

      fir->setup(vec);
      fir->setInputPort(IMAGE,"in", passthrough, "out");

      // IIR denominator
      FilterFIR* denom = getNode<FilterFIR>("iir_denom", cv::Point(500,350));
      static float arr2[] =  { 
                 1.7600418803, // y[n-1]
                -1.1828932620, // * y[n- 2])
                0.2780599176, // * y[n- 3])
                }; 

      vector<float> vec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );
      denom->setup(vec2);
      denom->setInputPort(IMAGE,"in",fir, "out");
       
      Add* add_iir = getNode<Add>("add_iir", cv::Point(400,100) );
      add_iir->out = test_im;
      vector<ImageNode*> add_in;
      add_in.push_back(fir);
      add_in.push_back(denom);
      vector<float> nf;
      nf.push_back(1.0);
      nf.push_back(1.0);
      add_iir->setup(add_in, nf);
      
      //output = add_iir;
    }
    #endif

    #if 0
    // make a chain, sort of a filter
    bool toggle = false;
    for (float ifr = advance; ifr <= 1.0; ifr += advance ) {

      Signal* s2 = getNode<Saw>("sawl", cv::Point(400.0 + ifr*400.0, 200.0 + ifr*40.0) );
      s2->setup(advance, ifr);

      Tap* p2 = getNode<Tap>("tapl", cv::Point(400.0 + ifr*410.0, 300.0 + ifr*30.0) );
      p2->setup(s2, cam_buf);
      p2->out = test_im;

      Add* add = getNode<Add>("addl", cv::Point(400.0 + ifr*430.0, 400.0 + ifr*10.0) );
      add->out = test_im;
      add->setup(nd, p2, toggle ? 1.5 : 0.5, toggle? -0.5 :0.5);
      toggle = !toggle;
      /*
         Signal* s3 = new Saw(advance, ifr -advance*2.5);
         Tap* p3 = new Tap(s2, cam_buf);

         add = new Add(add, p3, 2.0, -1.0);
         */
      nd = add;
    }
  #endif

    //output = nd;
    //output = p1;
/*
    cv::namedWindow("cam", CV_GUI_NORMAL);
    cv::moveWindow("cam", 0, 0);
*/
    output_node = (Output*)node;
    saveGraph("default_graph.yml");
  }

  void selectSourceNode() 
  {
    if (source_node != all_nodes[selected_ind]) {
      // select source node
      source_ind = selected_ind;
      source_node = all_nodes[source_ind];
      source_port = selected_port;

      source_type = selected_type;
      //if (source_type == "ImageNode") source_type = "ImageOut";
      //if (source_type == "ImageOut") source_type = "ImageNode";
      VLOG(1) << "selected source node " << source_type << " " << source_port; 
    } else {
      // select no source port, allows cycling through all inputs
      source_ind = 0;
      source_node = NULL;
      source_type = NONE;
      source_port = "";
      VLOG(1) << "cleared source node";
    }
  }

  bool selectTargetNode() 
  {
    // select the target node
    // connect to source node in best way possible, replacing the current input
    // TBD need to be able to select specific inputs
    if (source_node && selected_node && 
        (source_type != NONE) && (selected_port != "") && (source_port != "")) {

      // TBD a Buffer should act as an ImageNode if that is the only
      // input available
      selected_node->setInputPort(source_type, selected_port, source_node, source_port);
      
      //VLOG(1) << 
      return true;
    }  // legit source_node
    
    return false;
  }

  bool removePortConnection() 
  {
    // disconnect current input
    if (selected_node && (selected_type != NONE) && (selected_port != "")) {
      selected_node->setInputPort(selected_type, selected_port, NULL, "");
    }
  }

  void selectNextNode() 
  {
    //if (selected_node) selected_node->draw_selected_port = false;
    
    // move forward in selection
    selected_ind++;
    if (selected_ind >= all_nodes.size()) selected_ind = 0;
    selected_node = all_nodes[selected_ind];

    selected_port = "";
    selectPort(false);

    VLOG(1) << "selected node " << selected_node->name << " " << selected_ind;
  }

  void selectPrevNode()
  {
    //if (selected_node) selected_node->draw_selected_port = false;
    // move backward in selection
    selected_ind--;
    if (selected_ind < 0) selected_ind = all_nodes.size()-1;
    selected_node = all_nodes[selected_ind];

    selected_port = "";
    selectPort(false);
    
    VLOG(1) << "selected node " << selected_node->name << " " << selected_ind;
  }

  // TBD move into Node?
  bool selectPort(bool next_port = true)
  {
    if (!selected_node) {
      VLOG(1) << "selectPort: no selected_node";
      return false;
    }
    
    selected_node->draw_selected_port = true;

    // take the current port
    if (!next_port) {
      selected_type = selected_node->selected_type;
      selected_port = selected_node->selected_port;
      selected_port_ind = selected_node->selected_port_ind;
      return true;
    }

    bool rv = selected_node->getNextPort(source_type);
    
    if (rv) {
    selected_type = selected_node->selected_type;
    selected_port = selected_node->selected_port;
    selected_port_ind = selected_node->selected_port_ind;
    }

    return rv;
  } // selectPort()

  int key;
  bool valid_key;
  string command_text;
  bool draw_nodes;
  bool paused;

  bool handleInput() 
  {
    if (VLOG_IS_ON(10) || paused) 
      key = waitKey(0);
    else 
      key = waitKey(20);

    if (key < 0) return true;

    valid_key = true;
    if( key == 'q' ) {
      return false;
    }
    // TBD look for /, then make next letters type search for nodes with names container the following string
    else if (key == 'x' ) {
      paused = !paused;
    }
    else if( key == 'e' ) {
      // TBD follow with file name
      // TBD load the graph in a temp object, then copy it over only if successful
      LOG(INFO) << "reloading graph file";
      clearNodes();
      loadGraph(FLAGS_graph_file);
    }
    else if( key == 'w' ) {
      // TBD increment a count so old saves aren't overwritten?
      saveGraph("temp_graph.yml");
    }
    else if (key == 'a') {
      gridGraph();
    }
    else if (key == 'z') {
      draw_nodes = !draw_nodes;
    }
    else if (key == 's') {
      if (selected_node) selected_node->enable = !selected_node->enable;
    }
    
    // Connection manipulation
    else if (key == 'j') {
      selectNextNode();
    }
    else if (key == 'k') {
      selectPrevNode();
    }
    else if (key == 'u') {
      // TBD the node should catch this itself
      selectPort();
      
      stringstream str;
      if (selected_node) {
        str << selected_node->name << " : ";
        str << "matching " << source_type << " \"" << source_port << "\" with ";
      } else {
        str <<"selecting";
      }
      
      str 
          << selected_type << " \"" << selected_port << "\" "
          << CLTXT 
          << selected_ind << " " << selected_port_ind << " " 
          << CLNRM;
      VLOG(1) << str.str();
    } 
    else if (key == 'r') {
      selectSourceNode();
    } 
    else if (key == 't') {
      selectTargetNode();
    } // set source_node to target input
    else if (key == 'd') {
      removePortConnection();
    }
    //else if (key == 'c') {
      // swap selected node input with source node input- TBD this doesn't work since  
      // outputs can be one-to-many
    //}
    else {
      valid_key = false;
      // see if node can work with the key
      if (selected_node) 
        valid_key = selected_node->handleKey(key);
    }

    if (valid_key) {
      stringstream tmp;
      tmp << (char) key;
      //tmp.resize(1);
      //tmp[0] = key;
      command_text.append(tmp.str());
      VLOG(4) << tmp.str() << " " << command_text;
    } else if (key >= 0) {
      LOG(INFO) << "unused keypress:" << (char)key << " " << key;
    }

    int max_count = 24;
    if (command_text.size() > 40) max_count /= 2;
    if (command_text.size() > 80) max_count /= 2;
    if (command_text.size() > 160) max_count = 1;
    if (count % max_count == 0) {
      if (command_text.size() > 0);
      command_text = command_text.erase(0,1);
    } else {
      //command_text = "";
    }

    return true;
  }

  bool update() 
  {
    // TBD capture key input in separate thread?
    count++;
   
    // have to do this before update to avoid some crashes
    if (!handleInput()) return false;
 
    // TBD put this in different thread 
      { 
        VLOG(4) << "";
        if (!output_node) {
          LOG(ERROR) <<"no output_node";
          return false;
        }
        output_node->setUpdate();
        output_node->update();
      }

    return true;
  }
 
  void draw() 
  {
    graph_im = cv::Scalar(0,0,0);
    cv::Mat out_node_im = output_node->getImage("out");
    if (!out_node_im.empty()) {
      cv::resize(out_node_im * (draw_nodes ? 0.35 : 1.0),  graph_im,
          graph_im.size(), 0, 0, cv::INTER_NEAREST );
    }
    else
      graph_im = cv::Scalar(0,0,0);

    if (draw_nodes) {
      if (source_node && selected_node) {
        cv::line( graph_im, source_node->loc, selected_node->loc, cv::Scalar(70, 70, 70), 8, 4 );
      }

      cv::putText(graph_im, command_text, cv::Point(10, graph_im.rows-40), 1, 1, cv::Scalar(200,205,195), 1);
      if (command_text.size() > 0) { 
        VLOG(5) << "command_text " << command_text;
      }

      // TBD could all_nodes size have
      if (selected_node) {
        cv::circle(graph_im, selected_node->loc, 18, cv::Scalar(0,220,1), -1);
      }
      if (source_node) {
        cv::circle(graph_im, source_node->loc, 13, cv::Scalar(29,51,11), -1);
        cv::circle(graph_im, source_node->loc, 12, cv::Scalar(229,151,51), -1);
      }

      // draw input and outputs
      /*
         imshow("cam", cam_buf->get());

         cv::Mat out = output->get();
         if (out.data) {
      //imshow("out", out);
      } else {
      LOG(ERROR) << "out no data";
      }*/

      // loop through
      for (int i = 0; i < all_nodes.size(); i++) {
        all_nodes[i]->draw();
      }
    }

    imshow("graph_im", graph_im);
  }

  };

}

/*
 * To work with Kinect the user must install OpenNI library and PrimeSensorModule for OpenNI and
 * configure OpenCV with WITH_OPENNI flag is ON (using CMake).

TBD have a mode that takes a webcam, uses brightness as depth, and thresholds it for the valid map

 */
int main( int argc, char* argv[] )
{
 // google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::LogToStderr();
  google::ParseCommandLineFlags(&argc, &argv, false);
 
  bm::CamThing* cam_thing = new bm::CamThing();
  
  bool rv = true;
  while(rv) {
    rv = cam_thing->update();
    cam_thing->draw();
    cam_thing->clearAllNodeUpdates();
  }

  return 0;
}

