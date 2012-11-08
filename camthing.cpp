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

#include "camthing.h"

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <time.h>

#include <deque>
#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>
#include <gflags/gflags.h>

#include "config.h"
#include "nodes.h" 
#include "misc_nodes.h" 
#include "signals.h"
#include "filter.h"
#include "cluster.h"
#include "generate.h"
#include "screencap.h"
#include "output.h"
#include "input.h"
#include "structure.h"

using namespace cv;
using namespace std;

DEFINE_string(graph_file, "../graph.yml", "yaml file to load with graph in it");

//DEFINE_bool(
namespace bm {

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

class CamThing : public Output 
{
  // TBD informal timer for the system
  
  // make sure all Nodes are stored here
  deque<Node*> all_nodes;

  // the final output 
  // TBD make this a special node type
  Output* output_node;
  // TBD temp- currently need to connect output Xlib parameters to Mouse
  Mouse* input_node;

  // where the ui pointer is (currently controlled by keyboard
  cv::Point cursor;
  // scale factor on drawing ui elements
  float zoom;

  public:
  
  // where the upper left coordinate of the window into the ui is
  cv::Point2f ui_offset;

  // conveniently create and store node
  template <class nodeType>
    nodeType* getNode(string name = "", cv::Point2f loc=cv::Point2f(0.0, 0.0))
    {
      nodeType* node = new nodeType();//name, loc, graph_ui);
      
      VLOG(1) << CLVAL << all_nodes.size()  << CLTX2 
          << " new node " << CLNRM << " " << getId(node) << " "  
          << name << " " << loc.x << ", " << loc.y;

      node->name = name;
      node->loc = loc;
      node->graph_ui = graph_ui;

      all_nodes.push_back(node);

      return node;
    }

  
    Node* getNodeByName(string type_id, string name = "", cv::Point2f loc=cv::Point2f(0.0, 0.0))
    {
      Node* node = NULL;

      if (type_id.compare("bm::Webcam") == 0) {
        // TBD make a version of getNode that takes a type_id string
        Webcam* cam_in = getNode<Webcam>(name, loc);
        node = cam_in;

        test_im = cam_in->getImage("out").clone();
        test_im = cv::Scalar(200,200,200);

      } else if (type_id.compare("bm::CamThing") == 0) {
        // TBD  don't allow duplicate camthings
        VLOG(1) << CLVAL << all_nodes.size()  << CLTX2 
          << " new node " << CLNRM << name << " " << loc.x << ", " << loc.y;
        // the node already exists
        //node = getNode<ScreenCap>(name, loc);
        node = this;
        node->loc = loc;
        node->name = name;
        all_nodes.push_back(node);
      } else if (type_id.compare("bm::ScreenCap") == 0) {
        node = getNode<ScreenCap>(name, loc);
        node->update();
      } else if (type_id.compare("bm::ImageNode") == 0) {
        node = getNode<ImageNode>(name, loc);
      } else if (type_id.compare("bm::Sobel") == 0) {
        node = getNode<Sobel>(name, loc);
      } else if (type_id.compare("bm::GaussianBlur") == 0) {
        node = getNode<GaussianBlur>(name, loc);
      } else if (type_id.compare("bm::Buffer") == 0) {
        node = getNode<Buffer>(name, loc);
      } else if (type_id.compare("bm::Mux") == 0) {
        node = getNode<Mux>(name, loc);
      } else if (type_id.compare("bm::MuxBuffer") == 0) {
        node = getNode<MuxBuffer>(name, loc);
      } else if (type_id.compare("bm::FilterFIR") == 0) {
        node = getNode<FilterFIR>(name, loc);
      } else if (type_id.compare("bm::Cluster") == 0) {
        node = getNode<Cluster>(name, loc);
      } else if (type_id.compare("bm::ImageDir") == 0) {
        node = getNode<ImageDir>(name, loc);
      } else if (type_id.compare("bm::Add") == 0) {
        node = getNode<Add>(name, loc);
      } else if (type_id.compare("bm::Multiply") == 0) {
        node = getNode<Multiply>(name, loc);
      } else if (type_id.compare("bm::AbsDiff") == 0) {
        node = getNode<AbsDiff>(name, loc);
      } else if (type_id.compare("bm::Greater") == 0) {
        node = getNode<Greater>(name, loc);
      } else if (type_id.compare("bm::Resize") == 0) {
        node = getNode<Resize>(name, loc);
      } else if (type_id.compare("bm::Flip") == 0) {
        node = getNode<Flip>(name, loc);
      } else if (type_id.compare("bm::Rot2D") == 0) {
        node = getNode<Rot2D>(name, loc);
      } else if (type_id.compare("bm::Signal") == 0) {
        node = getNode<Signal>(name, loc);
      } else if (type_id.compare("bm::Saw") == 0) {
        node = getNode<Saw>(name, loc);
      } else if (type_id.compare("bm::SigBuffer") == 0) {
        node = getNode<SigBuffer>(name, loc);
      } else if (type_id.compare("bm::Tap") == 0) {
        node = getNode<Tap>(name, loc);
      } else if (type_id.compare("bm::TapInd") == 0) {
        node = getNode<TapInd>(name, loc);
      } else if (type_id.compare("bm::Contour") == 0) {
        node = getNode<Contour>(name, loc);
      } else if (type_id.compare("bm::Bezier") == 0) {
        node = getNode<Bezier>(name, loc);
      } else if (type_id.compare("bm::Circle") == 0) {
        node = getNode<Circle>(name, loc);
      } else if (type_id.compare("bm::Noise") == 0) {
        node = getNode<Noise>(name, loc);
      } else if (type_id.compare("bm::Random") == 0) {
        node = getNode<Random>(name, loc);
      } else if (type_id.compare("bm::Gaussian") == 0) {
        node = getNode<Gaussian>(name, loc);
      } else if (type_id.compare("bm::Mouse") == 0) {
        node = getNode<Mouse>(name, loc);

        input_node = (Mouse*) node;
        if (output_node) {
          input_node->display = output_node->display;
          input_node->win = output_node->win;
          input_node->opcode = output_node->opcode;
        }
      } else if (type_id.compare("bm::Output") == 0) {
        node = getNode<Output>(name, loc);
        output_node = (Output*)node;
        output_node->setup(Config::inst()->out_width, Config::inst()->out_height);
      
        // TBD need better way to share X11 info- Config probably
        if (input_node) {
          input_node->display = output_node->display;
          input_node->win = output_node->win;
          input_node->opcode = output_node->opcode;
        }
      } else {
        LOG(WARNING) << "unknown node type " << type_id << ", assuming imageNode";
        node = getNode<ImageNode>(name, loc);
      }

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
    input_node = NULL;
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
    if (node == NULL) return -1;

    
    for (int i = 0; i < all_nodes.size(); i++) {
      if (all_nodes[i] == node) {
        VLOG(3) << node->name << " " << i; 
        return i; 
      }
    }

    return -1;
  }
  
  

  /// save the graph to an output yaml file
  bool saveGraph(std::string graph_file="graph.yml") 
  {
    boost::mutex::scoped_lock l(update_mutex);

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
      VLOG(2) << all_nodes[i]->name << " saving " << all_nodes[i]->ports.size() << " inputs";
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
          int src_ind = -1;
          if (con->src) {
            src_port = con->src->name;
            // TBD this function is currently why this isn't in the Node::save()
            src_ind = getIndFromPointer(con->src->parent);  
          }

          fs << "src_ind" << src_ind;
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
    
    float dx = graph_ui.cols / (wd + 1.5);
    float dy = graph_ui.rows / (ht);
    
    LOG(INFO) << "making " << all_nodes.size() << " graph items into grid " << wd << " " << dx << " " << dy;

    for (int y = 0; y <= wd; y++) {
    for (int x = 0; x <= wd; x++) {
      const int ind = y*(wd+1) + x;
      if (ind >= all_nodes.size()) continue;
      
      cv::Point loc = cv::Point2f( x*dx + dx/4.0, y*dy + dy/4.0 );
      
      LOG(INFO) << ind << " " << getId(all_nodes[ind]) << " "
          <<  all_nodes[ind]->name << " " 
          << all_nodes[ind]->loc.x << " " << all_nodes[ind]->loc.y 
          << " -> " << loc.x << " " << loc.y ;
      
      all_nodes[ind]->loc = loc;

    }}
  }

  Webcam* cam_in;
  int count;

  cv::Mat test_im;

  cv::Mat cam_image;

  // TBD make struct for these two
  int selected_ind;
  Node* selected_node;
  //conType selected_port_type;
  //string selected_port;
  //int selected_port_ind;

  // store a node to be connected to a different selected node
  int source_ind;
  Node* source_node;
  // ImageNode, Signal, or Buffer for now- TBD use enums instead of strings
  conType source_type;
  string source_port;
  int source_port_ind;
  
  boost::thread node_thread;

  CamThing() : 
      selected_ind(0), 
      selected_node(NULL),
      //selected_port_type(NONE),
      //selected_port(""),
      //selected_port_ind(0),
      source_ind(0),
      source_node(NULL),
      source_type(NONE),
      source_port(""),
      source_port_ind(0),
      output_node(NULL),
      input_node(NULL),
      draw_nodes(true),
      paused(true)
  {
    count = 0;

    // TBD make internal type a gflag
    // or have a config file that loads it and resolution also
    graph_ui = cv::Mat(cv::Size(Config::inst()->ui_width, Config::inst()->ui_height), MAT_FORMAT_C3);
    graph_ui = cv::Scalar(0);


    //node->name = name;
    //node->loc = loc;


    if ((FLAGS_graph_file =="") || (!loadGraph(FLAGS_graph_file))) {
      defaultGraph();
    } 
    saveGraph("graph_load_test.yml");

    selectNextNode();

    // Xlib stuff
    {
      setup(graph_ui.size().width, graph_ui.size().height);

      // Try to get keyboard input working
      XIEventMask eventmask;
      unsigned char mask[1] = { 0 }; /* the actual mask */

      // only 1 worked for my laptop keyboard
      eventmask.deviceid = 1;
      eventmask.mask_len = sizeof(mask); /* always in bytes */
      eventmask.mask = mask;
      /* now set the mask */
      XISetMask(mask, XI_ButtonPress);
      XISetMask(mask, XI_Motion);
      XISetMask(mask, XI_KeyPress);

      /* select on the window */
      XISelectEvents(display, win, &eventmask, 1);
    }


    update_nodes = true;
    node_thread = boost::thread(&CamThing::updateThread, this);
  }

  ///////////////////////////////////////////////
  bool loadGraph(const std::string graph_file)
  {
    boost::mutex::scoped_lock l(update_mutex);
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
      
      Node* node = getNodeByName(type_id, name, loc);
      
      if (node == NULL) { 
        LOG(ERROR) << "couldn't get node made " << type_id << " " << name;
        continue;
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

      VLOG(1) << type_id << " " << CLTXT << name << CLVAL << " " 
          << node  << " " << loc << " " << enable << CLNRM;
      
      int ind;
      (*it)["ind"] >> ind;
      VLOG(1) << CLTXT << "first pass inputs " << CLVAL << ind << CLNRM << " " << node->name;

      for (int i = 0; i < (*it)["inputs"].size(); i++) {
        int type;
        string port;

        (*it)["inputs"][i]["type"] >> type;
        (*it)["inputs"][i]["name"] >> port;
      
        VLOG(1) << "input " << ind << " \"" << node->name
            << "\", type " << type << " " << port;
        
        // TBD make function for this
        /*
        conType con_type = NONE;
        if (type == "ImageNode") con_type = IMAGE;
        if (type == "ImageOut") con_type = IMAGE;
        if (type == "Signal") con_type = SIGNAL;
        if (type == "Buffer") con_type = BUFFER;
        */
        
        node->setInputPort((conType)type, port, NULL, "");
      }
    }

    // second pass for inputs (the first pass was necessary to create them 
    // all in right order
    VLOG(1) << " ";
    LOG(INFO) << "second pass inputs";
    VLOG(1) << " ";
    for (FileNodeIterator it = nd.begin(); it != nd.end(); ++it) {
      int ind;
      (*it)["ind"] >> ind;
      VLOG(1) << "second pass inputs " << ind << " " << CLTXT << all_nodes[ind]->name << CLNRM;
      for (int i = 0; i < (*it)["inputs"].size(); i++) {
        int input_ind;
        int type;
        string port;
        string src_port;
        float value;

        (*it)["inputs"][i]["type"] >> type;
        (*it)["inputs"][i]["name"] >> port;
        (*it)["inputs"][i]["src_ind"] >> input_ind;
        (*it)["inputs"][i]["src_port"] >> src_port;
        (*it)["inputs"][i]["value"] >> value;
        
        if (input_ind >= 0) {
        VLOG(1) << "input " 
            << " " << input_ind << ", type " << type << " " << port << " " << input_ind
            << " " << src_port;
      
          all_nodes[ind]->setInputPort((conType)type, port, all_nodes[input_ind], src_port);
        } // input_ind > 0

        if (type == SIGNAL) {
          
          all_nodes[ind]->setSignal(port, value);
        }

      }
    } // second input pass

    if (output_node == NULL) {
      LOG(WARNING) << CLWRN << "No output node found, setting it to " 
          << all_nodes[all_nodes.size() - 1]->name << CLNRM;
      // TBD could make sure that this node is an output node
      
      output_node = (Output*) all_nodes[all_nodes.size() - 1];
    }

    LOG(INFO) << all_nodes.size() << " nodes total";
    //output_node->loc = cv::Point2f(graph.cols - (test_im.cols/2+100), 20);
   
    return true;

  } // loadGraph

  void defaultGraph() 
  {
    LOG(INFO) << "creating default graph";
    Node* node;
    
    // create a bunch of nodes of every type (TBD make sure new ones get added)
    // but don't connect them, user will do that
    
    cv::Point2f loc = cv::Point2f(400,400);
    
    node = this;
    node->loc = loc;
    node->name = "cam_thing";
    all_nodes.push_back(node);

    // Images
    // inputs
    //node = getNode<Webcam>("web_cam", loc);
    //node->update();

    node = getNode<ScreenCap>("screen_cap", loc);
    //node->update();

    ImageDir* im_dir = getNode<ImageDir>("image_dir", loc);
    im_dir->dir = "../data/";
    im_dir->loadImages();

    // process
    getNode<Buffer>("buffer", loc);
    getNode<Mux>("mux", loc);
    getNode<MuxBuffer>("mux_buffer", loc);
    getNode<FilterFIR>("filter_fir", loc);
    getNode<Cluster>("cluster", loc);

    node = getNode<Sobel>("sobel", loc);
    node = getNode<GaussianBlur>("blur", loc);
    node = getNode<Rot2D>("rot2d", loc);
    node = getNode<Resize>("resize", loc);
    node = getNode<Flip>("flip", loc);

    node = getNode<Add>("add", loc);
    node = getNode<Multiply>("multiply", loc);
    node = getNode<AbsDiff>("abs_diff", loc);
    node = getNode<Greater>("greater", loc);
    
    // generate
    node = getNode<Bezier>("bezier", loc);
    node = getNode<Circle>("circle", loc);
    node = getNode<Noise>("noise", loc);

    node = getNode<Tap>("tap0", loc);
    node = getNode<TapInd>("tapind0", loc);

    // structure
    node = getNode<Contour>("contour", loc);

    // Signals
    // inputs
    node = getNode<Mouse>("mouse", loc);
    input_node = (Mouse*) node;

    // generate
    node = getNode<Signal>("signal0", loc);
    node = getNode<Saw>("saw", loc);
    node = getNode<Random>("random", loc);
    node = getNode<Gaussian>("gaussian", loc);
    
    node = getNode<SigBuffer>("sig_buf", loc);
    
    node = getNode<Output>("output", loc);
    output_node = (Output*)node;
    output_node->setup(Config::inst()->out_width, Config::inst()->out_height);

    // TBD need better way to share X11 info- Config probably
    if (input_node) {
      input_node->display = output_node->display;
      input_node->win = output_node->win;
      input_node->opcode = output_node->opcode;
    }

    gridGraph(); 

  #if MAKE_FIR
    {
      FilterFIR* fir = getNode<FilterFIR>("fir_filter", cv::Point2(500,150));

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
      FilterFIR* denom = getNode<FilterFIR>("iir_denom", cv::Point2f(500,350));
      static float arr2[] =  { 
                 1.7600418803, // y[n-1]
                -1.1828932620, // * y[n- 2])
                0.2780599176, // * y[n- 3])
                }; 

      vector<float> vec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );
      denom->setup(vec2);
      denom->setInputPort(IMAGE,"in",fir, "out");
       
      Add* add_iir = getNode<Add>("add_iir", cv::Point2f(400,100) );
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

      Signal* s2 = getNode<Saw>("sawl", cv::Point2f(400.0 + ifr*400.0, 200.0 + ifr*40.0) );
      s2->setup(advance, ifr);

      Tap* p2 = getNode<Tap>("tapl", cv::Point2f(400.0 + ifr*410.0, 300.0 + ifr*30.0) );
      p2->setup(s2, cam_buf);
      p2->out = test_im;

      Add* add = getNode<Add>("addl", cv::Point2f(400.0 + ifr*430.0, 400.0 + ifr*10.0) );
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

  bool selectSourceNode() 
  {

    if (selected_node && (source_node != selected_node)) {
      // select source node
      source_ind = selected_ind;
      source_node = selected_node; //all_nodes[source_ind];
      source_port = selected_node->selected_port;

      source_type = selected_node->selected_type;
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

  // make a connection
  bool selectTargetNode() 
  {
    boost::mutex::scoped_lock l(update_mutex);
    // select the target node
    // connect to source node in best way possible, replacing the current input
    // TBD need to be able to select specific inputs
    if (source_node && selected_node && 
        (source_type != NONE) && 
        (selected_node->selected_port != "") && 
        (source_port != "") && 
        (selected_node->selected_port != "") && 
        (selected_node->selected_port != "out")
        ) {

      // TBD a Buffer should act as an ImageNode if that is the only
      // input available
      selected_node->setInputPort(source_type, selected_node->selected_port, source_node, source_port);
      
      //VLOG(1) << 
      return true;
    }  // legit source_node
    
    return false;
  }

  bool removePortConnection() 
  {
    boost::mutex::scoped_lock l(update_mutex);

    // disconnect current input
    if (selected_node && (selected_node->selected_type != NONE) && (selected_node->selected_port != "")) {
      selected_node->setInputPort(selected_node->selected_type, selected_node->selected_port, NULL, "");
    }
  }

  void selectNextNode() 
  {
    //if (selected_node) selected_node->draw_selected_port = false;
    
    // move forward in selection
    selected_ind++;
    if (selected_ind >= all_nodes.size()) selected_ind = 0;
    selected_node = all_nodes[selected_ind];

    //selected_port = "";
    selectPort(0);

    VLOG(1) << "selected node " << selected_node->name << " " << selected_ind;
  }

  void selectPrevNode()
  {
    //if (selected_node) selected_node->draw_selected_port = false;
    // move backward in selection
    selected_ind--;
    if (selected_ind < 0) selected_ind = all_nodes.size()-1;
    selected_node = all_nodes[selected_ind];

    //selected_port = "";
    selectPort(0);
    
    VLOG(1) << "selected node " << selected_node->name << " " << selected_ind;
  }

  bool selectPortSource() 
  {
    VLOG(1) << "selecting port source";
    // jump to the node and port inputting into this port
    if (!selected_node) return false;
    if (selected_node->selected_port_ind < 0) return false;
    if (selected_node->selected_port_ind >= selected_node->ports.size()) return false;

    Connector* con = selected_node->ports[selected_node->selected_port_ind];
    if (!con->src) return false;

    selected_node = con->src->parent;
    selected_ind = getIndFromPointer(selected_node); 

    selected_node->selected_port_ind = selected_node->getIndFromPointer(con->src);

    // tell the node to select the port
    selected_node->selectPortByInd(selected_node->selected_port_ind);
    // keep a copy of the selected port local (TBD)
    selectPort(0);
    
    VLOG(2) << "selecting port source success";
    return true;
  }

  bool selectPortDestination() 
  {
    VLOG(1) << "selecting port destination";
    // jump to the node and port inputting into this port
    if (!selected_node) return false;
    if (selected_node->selected_port_ind < 0) return false;
    if (selected_node->selected_port_ind >= selected_node->ports.size()) return false;

    Connector* con = selected_node->ports[selected_node->selected_port_ind];
    if (!con->dst) return false;
    
    const string src = con->parent->name;

    selected_node = con->dst->parent;
    selected_ind = getIndFromPointer(selected_node); 

    selected_node->selected_port_ind = selected_node->getIndFromPointer(con->dst);

    // tell the node to select the port
    selected_node->selectPortByInd(selected_node->selected_port_ind);
    // keep a copy of the selected port local (TBD)
    selectPort(0);

    const string dst = selected_node->name;
    VLOG(2) << "selecting port destination success: " << src << " to " << dst;
    return true;
  }

  /*
    TBD move into Node?
    upper port = -1
    same port = 0
    lower port = 1
  */
  bool selectPort(int next_port = 1)
  {
    if (!selected_node) {
      VLOG(1) << "selectPort: no selected_node";
      return false;
    }
    
    selected_node->draw_selected_port = true;

    // take the current port
    if (next_port == 0) {
      //selected_port_type = selected_node->selected_type;
      //selected_port      = selected_node->selected_port;
      //selected_port_ind  = selected_node->selected_port_ind;
      return true;
    }

    bool rv;
    if (next_port > 0) 
      rv = selected_node->getNextPort(source_type);
    if (next_port < 0)
      rv = selected_node->getPrevPort(source_type);
    
    if (rv) {
      //selected_port_type = selected_node->selected_type;
      //selected_port = selected_node->selected_port;
      //selected_port_ind = selected_node->selected_port_ind;
    }

    return rv;
  } // selectPort()

  int key;
  bool valid_key;
  string command_text;
  bool draw_nodes;
  bool paused;


  /*
   *  -2 is  up
   *  2 is down
   *  -1 is left
   *  1 is right
   */
  int getNodeDirection(const int dir) 
  {
      // find closest node in y
      float cur_y = 0;
      if (selected_node) cur_y = selected_node->loc.y;
      float cur_x = 0;
      if (selected_node) cur_x = selected_node->loc.x;
     
      int min_ind = -1;
      //float min_dx = dir*1e6;
      //float min_dy = dir*1e6;
      float min_len = 1e6;

      for (int i = 0; i < all_nodes.size(); i++) {
        if (i == selected_ind) continue;

        float dy = all_nodes[i]->loc.y - cur_y;
        float dx = all_nodes[i]->loc.x - cur_x;
        float len = dx*dx + dy*dy;
        bool test = false;
        
        if (dir == -2) 
          test = ((abs(dx) <= abs(dy)) && (dy <= 0) );
        if (dir == -1) 
          test = ((abs(dy) <= abs(dx)) && (dx <= 0) );
        if (dir ==  1) 
          test = ((abs(dy) <= abs(dx)) && (dx > 0) );
        if (dir ==  2) 
          test = ((abs(dx) <= abs(dy)) && (dy > 0) );

        if (test && (len < min_len)) {
          //min_dy = dy;
          //min_dx = dx;
          min_len = len;
          min_ind = i;
        }
      }

    return min_ind;
  }

  bool selectNodeDirection(const int dir) 
  {
    int min_ind = getNodeDirection(dir);
      
      if (min_ind >= 0) {
        selected_node = all_nodes[min_ind];
        selected_ind = min_ind;
        return true;
      }
      return false;
  }
  
  bool handleInput() 
  {
    count++;

    key = -1;
    if (!display) return false;

      while (XPending(display)) {

      XKeyPressedEvent key_data;
      XEvent ev;
      /* Get next event; blocks until an event occurs */
      XNextEvent(display, &ev);
      if (ev.xcookie.type == GenericEvent &&
          ev.xcookie.extension == opcode &&
          XGetEventData(display, &ev.xcookie))
      //if (XCheckWindowEvent(display, win, PointerMotionMask | ButtonPressMask | ButtonReleaseMask, &ev))
      {
        VLOG(2) <<" event found"; 
        XIDeviceEvent* evData = (XIDeviceEvent*)(ev.xcookie.data);
        int deviceid = evData->deviceid;

        switch(ev.xcookie.evtype)
        {
          case XI_Motion:
            //LOG(INFO) <<  "motion";
            setSignal(boost::lexical_cast<string>(deviceid) + "_x", evData->event_x);
            setSignal(boost::lexical_cast<string>(deviceid) + "_y", evData->event_y);
            VLOG(2) << deviceid << " " << evData->event_x << " " << evData->event_y;

            break;

          case XI_ButtonPress:
            VLOG(2) << deviceid << " button: " << evData->detail;
            setSignal(boost::lexical_cast<string>(deviceid) + "_" + 
                boost::lexical_cast<string>(evData->detail), 1);

            break;

          case XI_ButtonRelease:
            VLOG(2) << deviceid << " unclick " << evData->detail;
            setSignal(boost::lexical_cast<string>(deviceid) + "_" + 
                boost::lexical_cast<string>(evData->detail), 0);
            break;
          
          case XI_KeyPress:
            // Assign info from our XIDeviceEvent to a standard XKeyPressedEvent which
            // XLookupString() can actually understand:
            key_data.type         = KeyPress;
            key_data.root         = evData->root;
            key_data.window       = evData->event;
            key_data.subwindow    = evData->child;
            key_data.time         = evData->time;
            key_data.x            = evData->event_x;
            key_data.y            = evData->event_y;
            key_data.x_root       = evData->root_x;
            key_data.y_root       = evData->root_y;
            key_data.same_screen  = True;        
            key_data.send_event   = False;
            key_data.serial       = evData->serial;
            key_data.display      = display;

            key_data.keycode      = evData->detail;
            key_data.state        = evData->mods.effective;   
           
            char asciiChar;
            if (1 == XLookupString(((XKeyEvent*) (&key_data)), &asciiChar, 1, NULL, NULL)) {
              // Mapped: Assign it.
              key = (int) asciiChar;
              VLOG(2) << deviceid << "key down" << key << " " << (char) key;
            }
            break;

          case XI_KeyRelease:
            VLOG(2) << deviceid << "key up" << evData->detail;
            break;
        } // switch 
      } // correct event

      XFreeEventData(display, &ev.xcookie);
  
      //usleep(1000); 
    } // while

    /*
    if (VLOG_IS_ON(10) || paused) 
      key = waitKey(0);
    else 
      key = waitKey(30);
      */

    if (key < 0) return true;

    valid_key = true;
    if( key == 'q' ) {
      update_nodes = false;
      node_thread.join();
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
    //else if (key == 'a') {
    //  gridGraph();
    //}
    else if (key == 'z') {
      draw_nodes = !draw_nodes;
    }
    else if (key == 'a') {
      if (selected_node) {
        // TBD make node function to do this without exposing setSignal 
        selected_node->setSignal("enable",  !((bool)selected_node->getSignal("enable")));
      }
    }
    else if (key == ']') {
      // duplicate selected_node node
      // TBD make it possible to deselect all nodes, so the node specific keys can be reused?
      if (selected_node == this) {
        LOG(WARNING) << "can't duplicate camthing";
      }
      else if (selected_node) {
        LOG(INFO) << " duplicating selected_node";
        getNodeByName(getId(selected_node), selected_node->name + "_2", selected_node->loc + cv::Point2f(100,10));
      }
    }
    
    // Connection manipulation
    // jump back to selecting the source node
    else if (key == 'b') {
      if (source_node) {
        selected_node = source_node;
        selected_ind = source_ind;
        selectPort(0);
      }
    }
    /*
    else if (key == 'j') {
      selectNextNode();
    }
    else if (key == 'k') {
      selectPrevNode();
    }*/
    else if (key == 's') {
      selectPortSource();
    }
    else if (key == 'g') {
      // jump to the node and port this port is outputting to
      selectPortDestination();
    }
    else if (key == 'd') {
      // select the above port
      selectPort(-1);
    }
    else if (key == 'f') {
      // select the next port down
      // TBD the node should catch this itself
      const bool rv = selectPort(1);
      if (rv) { 
        stringstream str;
        if (selected_node) {
          str << selected_node->name << " : ";
          str << "matching " << source_type << " \"" << source_port << "\" with ";
        } else {
          str <<"selecting";
        }

        str 
          << selected_node->selected_type << " \"" << selected_node->selected_port << "\" "
          << CLTXT 
          << selected_ind << " " << selected_node->selected_port_ind << " " 
          << CLNRM;
        VLOG(1) << str.str();
      }
    } 
    else if (key == 'r') {
      selectSourceNode();
    } 
    else if (key == 't') {
      selectTargetNode();
    } // set source_node to target input
    else if (key == 'c') {
      removePortConnection();
    }
    //////////////////////////////////////////////////
    // following may not be portable
    else if (key == 'k') {  // UP
      selectNodeDirection(-2); 
    } else if (key == 'j') {  // DOWN
      selectNodeDirection(2); 
    } else if (key == 'h') {  // LEFT
      selectNodeDirection(-1); 
    } else if (key == 'l') {  // RIGHT
      selectNodeDirection(1); 
     
    } else if (key == 'i') {  // UP
      ui_offset -= cv::Point2f(0,15);
      VLOG(3) << "ui_offset " << ui_offset;
    } else if (key == 'u') {  // DOWN
      ui_offset += cv::Point2f(0,15); 
      VLOG(3) << "ui_offset " << ui_offset;
    } else if (key == 'y') {  // LEFT
      ui_offset -= cv::Point2f(15,0); 
      VLOG(3) << "ui_offset " << ui_offset;
    } else if (key == 'o') {  // RIGHT
      ui_offset += cv::Point2f(15,0); 
      VLOG(3) << "ui_offset " << ui_offset;
    
    //else if (key == 'c') {
      // swap selected node input with source node input- TBD this doesn't work since  
      // outputs can be one-to-many
    //}
    } else {
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

  bool update_nodes;
  boost::mutex update_mutex;

  bool updateThread() 
  {
    LOG(INFO) << "starting node update thread";
    while (update_nodes) {

      VLOG(4) << "";
      if (!output_node) {
        LOG_FIRST_N(ERROR,3) <<"no output_node";
        usleep(20000);
      }

      if (!paused) {
        boost::mutex::scoped_lock l(update_mutex);
        output_node->setUpdate();
        output_node->update();
        clearAllNodeUpdates();
      } else {
        usleep(10000);
      }
      usleep(1000);
    }
    LOG(INFO) << "ending node update thread";
    return true;
  }

#if 1
  virtual bool update() 
  {
    //ImageNode::update();
    Node::update();

    cv::Mat out = getImage("out");
    if (out.empty()) out = cv::Mat(cv::Size(Config::inst()->im_width, Config::inst()->im_height), MAT_FORMAT_C3);
    cv::resize(graph_ui, out,
          out.size(), 0, 0, cv::INTER_NEAREST );
    setImage("out", out);          
    return true;
  }
#endif

  virtual bool draw(cv::Point2f ui_offset) 
  {
    boost::timer t1;
    
    if (graph_ui.empty()) {
      LOG(ERROR) << "graph_ui empty";
      return false;
    }

    cv::Mat out_node_im = output_node->getImage("out").clone();
    if (false) { //(!out_node_im.empty()) {
      cv::resize(out_node_im * (draw_nodes ? 0.2 : 1.0),  graph_ui,
          graph_ui.size(), 0, 0, cv::INTER_NEAREST );
    }
    else
      graph_ui = cv::Scalar(0,0,0);
    
    
    if (!ImageNode::draw(ui_offset)) {
    //if (!Node::draw(ui_offset)) {
      LOG(ERROR) << "something wrong with node drawing";
      return false;
    }

   
    VLOG(3) << "bg draw time" << t1.elapsed(); 

    if (draw_nodes) {
      if (source_node && selected_node) {
        cv::line( graph_ui, 
            source_node->loc + ui_offset, selected_node->loc + ui_offset, 
            cv::Scalar(70, 70, 70), 8, 4 );
      }

      // draw potential neighbors to jump to with node selection commands
      if (selected_node) {
        int n_ind;

        for (int i = -2; i <= 2; i++) {
          n_ind = getNodeDirection(i);
          if (n_ind < 0) continue;
          cv::line( graph_ui, 
            all_nodes[n_ind]->loc + ui_offset, selected_node->loc + ui_offset, 
            cv::Scalar(30 + i*2, 50 - i*5, 40 + i * 10), 8, 4 );
        }
      }

      cv::putText(graph_ui, command_text, cv::Point2f(10, graph_ui.rows-40), 1, 1, 
          cv::Scalar(200,205,195), 1);
      if (command_text.size() > 0) { 
        VLOG(5) << "command_text " << command_text;
      }

      // TBD could all_nodes size have
      if (selected_node) {
        cv::circle(graph_ui, selected_node->loc + ui_offset, 18, cv::Scalar(0,220,1), -1);
      }
      if (source_node) {
        cv::circle(graph_ui, source_node->loc + ui_offset, 13, cv::Scalar(29,51,11), -1);
        cv::circle(graph_ui, source_node->loc + ui_offset, 12, cv::Scalar(229,151,51), -1);
      }
      VLOG(4) << "cv draw time" << t1.elapsed(); 

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
        if (all_nodes[i] != this)
          all_nodes[i]->draw(ui_offset);
      }
      VLOG(4) << "node draw time " << t1.elapsed();
    }

    // commenting this out breaks the drawing, not sure why
    setImage("in", graph_ui );
    VLOG(4) << "ui draw time" << t1.elapsed(); 
    
    Output::draw(ui_offset);

    VLOG(4) << "full draw time" << t1.elapsed(); 
  } // CamThing::draw

  };

}

#include <linux/input.h>
#include <fcntl.h>

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
 
// TEMP mouse test
  #ifdef MOUSE_TEST
  int fd;
  // cat /proc/bus/input/devices - TBD how to find just the mouses?
  // - TBD how to find just the mouses?
  // TBD can't get touchpad to work, don't even see it when catting /dev/input/mouseN
  if ((fd = open("/dev/input/event4", O_RDONLY)) < 0) {
    LOG(ERROR) << "couldn't open mouse " << fd;
    exit(0);
  }
  struct input_event ev;
  #else 
  bm::CamThing* cam_thing = new bm::CamThing();
  #endif


  bool rv = true;
  while(rv) {

    #ifdef MOUSE_TEST
    read(fd, &ev, sizeof(struct input_event));
    VLOG(1) << "value 0x" << std::hex << ev.value 
      << ", type 0x" << std::hex << ev.type 
      << ", code 0x" << std::hex << ev.code;
    if (ev.type == EV_REL) {
      if (ev.value != 0) {
        // mouse move left
        if (ev.code == ABS_X) LOG(INFO)<< "dx " << ev.value;
        // mouse move right
        if (ev.code == ABS_Y)  LOG(INFO)<< "dy " << ev.value;
        // wheel
        if (ev.code == REL_WHEEL) LOG(INFO)<< "wheel " << ev.value;
      }
    }
    if (ev.type == EV_MSC) {
      // 0x90001 - 3
      LOG(INFO) << "Button value 0x" << std::hex << ev.value 
      << ", code " << ev.code;

    }
    #else
    // have to do this before update to avoid some crashes
    // input is handled in the same thread as drawing because of Xlib- TBD
    rv = cam_thing->handleInput();
    cam_thing->draw(cam_thing->ui_offset);
    #endif
  }

  return 0;
}

