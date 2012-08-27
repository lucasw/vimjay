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
#include "screencap.h"

using namespace cv;
using namespace std;

DEFINE_string(graph_file, "../graph.yml", "yaml file to load with graph in it");
//DEFINE_bool(
namespace bm {

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
      nodeType* node = new nodeType();

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
      // TBD put in Node::load method
      for (inputsType::iterator it = all_nodes[i]->inputs.begin();
          it != all_nodes[i]->inputs.end(); it++)
      {
        for (inputsItemType::iterator it2 = it->second.begin();
            it2 != it->second.end(); it2++)
        {
          Node* cur_node = it2->second.first;
          if (cur_node == NULL) continue;

          fs << "{";
          fs << "type" << it->first;
          fs << "port" << it2->first;
          fs << "src_port" << it2->second.second;
          fs << "ind" << getIndFromPointer(cur_node);   
          fs << "}";
        }
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
 
  int selected_ind;
  Node* selected_node;
  string selected_type;
  string selected_port;
  int selected_port_ind;

  // store a node to be connected to a different selected node
  int source_ind;
  Node* source_node;
  // ImageNode, Signal, or Buffer for now- TBD use enums instead of strings
  string source_type;
  string source_port;
  
  CamThing() : 
      selected_ind(0), 
      selected_node(NULL),
      selected_type(""),
      selected_port(""),
      selected_port_ind(0),
      source_ind(0),
      source_node(NULL),
      source_type(""),
      source_port(""),
      output_node(NULL),
      draw_nodes(true),
      paused(false)
  {
    count = 0;

    // TBD make internal type a gflag
    // or have a config file that loads it and resolution also
    graph_im = cv::Mat(cv::Size(1280, 720), MAT_FORMAT_C3);
    graph_im = cv::Scalar(0);

    if (FLAGS_graph_file =="") {
      defaultGraph();
    } else {
      loadGraph(FLAGS_graph_file);
      saveGraph("graph_load_test.yml");
    }

    cv::namedWindow("graph_im", CV_GUI_NORMAL);
    cv::moveWindow("graph_im", 0, 500);
    cv::resizeWindow("graph_im", 1280, 720);
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
      }
      else if (type_id.compare("bm::ScreenCap") == 0) {
        node = getNode<ScreenCap>(name, loc);
        node->update();
      }
      else if (type_id.compare("bm::ImageNode") == 0) {
        node = getNode<ImageNode>(name, loc);
      }
      else if (type_id.compare("bm::Sobel") == 0) {
        node = getNode<Sobel>(name, loc);
      }
      else if (type_id.compare("bm::Buffer") == 0) {
        node = getNode<Buffer>(name, loc);
      }
      else if (type_id.compare("bm::ImageDir") == 0) {
        node = getNode<ImageDir>(name, loc);
      }
      else if (type_id.compare("bm::Add") == 0) {
        node = getNode<Add>(name, loc);
      }
      else if (type_id.compare("bm::Resize") == 0) {
        node = getNode<Resize>(name, loc);
      }
      else if (type_id.compare("bm::Rot2D") == 0) {
        node = getNode<Rot2D>(name, loc);
      }
      else if (type_id.compare("bm::Signal") == 0) {
        node = getNode<Signal>(name, loc);
      }
      else if (type_id.compare("bm::Saw") == 0) {
        node = getNode<Saw>(name, loc);
      }
      else if (type_id.compare("bm::Tap") == 0) {
        node = getNode<Tap>(name, loc);
      }
      else if (type_id.compare("bm::TapInd") == 0) {
        node = getNode<TapInd>(name, loc);
      }
      else if (type_id.compare("bm::Output") == 0) {
        node = getNode<Output>(name, loc);
        output_node = (Output*)node;
      } else{
        LOG(WARNING) << "unknown node type " << type_id << ", assuming imageNode";
        node = getNode<ImageNode>(name, loc);
      }

      if (dynamic_cast<ImageNode*>(node)) {
        (dynamic_cast<ImageNode*> (node))->setImage("out", test_im);
      }
      node->load(it);
      
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
        
        if (input_ind >= 0)
          all_nodes[ind]->setInputPort(type, port, all_nodes[input_ind], src_port);
        else 
          all_nodes[ind]->setInputPort(type, port, NULL, src_port);
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

    node = getNode<ImageNode>("out", loc);
    cv::Mat tmp;
    node->setImage("in", tmp); 


    //node = getNode<Output>("out", loc);

    gridGraph(); 

  #if 0
    ///////////////
    const float advance = 0.1;

    cam_in = getNode<Webcam>("webcam", cv::Point(50, 20) );
    test_im = cam_in->getImage("out").clone();
    test_im = cv::Scalar(200,200,200);

    ImageNode* passthrough = getNode<ImageNode>("image_node_passthrough", cv::Point(400, 50) );
    passthrough->setInputPort("ImageNode","in", cam_in, "out");
    passthrough->setImage("out", test_im);
    //passthrough->out_old = test_im;
    //output = passthrough;

    Add* add_loop = getNode<Add>("add_loop", cv::Point(600,100) );
    add_loop->setImage("out", test_im);

    Rot2D* rotate = getNode<Rot2D>("rotate", cv::Point(400,400));
    rotate->setInputPort("ImageNode","in", add_loop, "out");
    rotate->setImage("out", test_im);
    //rotate->out_old = test_im;
    rotate->setSignal("angle", 50.0);
    rotate->setSignal("center_x", test_im.cols/2);
    rotate->setSignal("center_y", test_im.rows/2);
    //output = rotate;

    Signal* sr = getNode<Saw>("saw_rotate", cv::Point(350,400) ); 
    sr->setup(0.02, -5.0, -5.0, 5.0);
    rotate->setInputPort("Signal","angle", sr, "out");

    Signal* scx = getNode<Saw>("saw_center_x", cv::Point(350,450) ); 
    scx->setup(5, test_im.cols/2, 0, test_im.cols);
    rotate->setInputPort("Signal","center_x", scx, "out");
    
    Signal* scy = getNode<Saw>("saw_center_y", cv::Point(350,500) ); 
    //scy->setup(6, test_im.rows/2, test_im.rows/2 - 55.0, test_im.rows/2 + 55.0);
    scy->setup(6, test_im.rows/2, 0, test_im.rows);
    rotate->setInputPort("Signal","center_y", scy , "out");

    vector<ImageNode*> in;
    in.push_back(passthrough);
    in.push_back(rotate);
    vector<float> nf;
    nf.push_back(0.1);
    nf.push_back(0.89);
    add_loop->setup(in, nf);
    #if 0
    if (false) {
    // test dead branch (shouldn't be updated)
    ImageNode* passthrough2 = getNode<ImageNode>("image_node_passthrough2", cv::Point(400, 450) );
    passthrough2->inputs.push_back(cam_in);
    passthrough2->out = test_im;
    passthrough2->out_old = test_im;
    
    output = passthrough2;
    }

    if (false) {

      // buffer test
      Buffer* cam_buf = getNode<Buffer>("buffer", cv::Point(500,50) );  
      cam_buf->max_size = ( 60 );
      cam_buf->out = test_im;

      output = cam_buf;

      if (false) {
        Add* add = getNode<Add>("addloop", cv::Point(50,500) );
        add->out = test_im;
        add->setup(passthrough, cam_buf, 0.8, 0.2);

        cam_buf->inputs.push_back(add);
      } else {
        cam_buf->inputs.push_back(passthrough);
      }

    
    Signal* s1 = getNode<Saw>("saw", cv::Point(500,400) ); 
    s1->setup(advance, 0);
    //Signal* s1 = getNode<Signal>("fixed signal", cv::Point(300,50) ); 
    //s1->setup(advance, 0);

    Tap* p1 = getNode<Tap>("tap", cv::Point(500,450) );
    //static_cast<Tap*>
    p1->setup(s1, cam_buf);
    p1->out = test_im;

    output = p1;
    } else
    #endif

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
      fir->setInputPort("ImageNode","in", passthrough, "out");

      // IIR denominator
      FilterFIR* denom = getNode<FilterFIR>("iir_denom", cv::Point(500,350));
      static float arr2[] =  { 
                 1.7600418803, // y[n-1]
                -1.1828932620, // * y[n- 2])
                0.2780599176, // * y[n- 3])
                }; 

      vector<float> vec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );
      denom->setup(vec2);
      denom->setInputPort("ImageNode","in",fir, "out");
       
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
   #endif 
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
      if (source_type == "ImageOut") source_type = "ImageNode";
    
    } else {
      // select no source port, allows cycling through all inputs
      source_ind = 0;
      source_node = NULL;
      source_type = "";
      source_port = "";
    }
  }

  bool selectTargetNode() 
  {
    // select the target node
    // connect to source node in best way possible, replacing the current input
    // TBD need to be able to select specific inputs
    if (source_node && selected_node && 
        (source_type != "") && (selected_port != "") && (source_port != "")) {

      // TBD a Buffer should act as an ImageNode if that is the only
      // input available
      selected_node->setInputPort(source_type,selected_port, source_node, source_port);
      
      //VLOG(1) << 
      return true;
    }  // legit source_node
    
    return false;
  }

  bool removePortConnection() 
  {
    // disconnect current input
    if (selected_node && (selected_type != "") && (selected_port != "")) {
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
      return true;
    }

    // loop through all inputs, unconstrained by source_type
    if (source_type == "") {
      vector<pair<string, string> > strn = selected_node->getInputStrings();

      if (strn.size() == 0) {
        VLOG(2) << "selectPort: " << CLTXT << selected_node->name 
            << CLNRM << " no input strings";
        return false;
      }
      
      int ind = 0;
      for (int i = 0; i < strn.size()-1; i++) {
        if ((strn[i].first == selected_type) &&
            (strn[i].second == selected_port)) {
          // select next one
          selected_type = strn[i+1].first;
          selected_port = strn[i+1].second;
          selected_port_ind = ind + 1;
          selected_node->selected_type = selected_type;
          selected_node->selected_port = selected_port;

          VLOG(2) << "selectPort: all inputs select";
          return true;
        }
        ind++;
      }
  
      selected_type = strn[0].first;
      selected_port = strn[0].second;
      selected_node->selected_type = selected_type;
      selected_node->selected_port = selected_port;
      selected_port_ind = 0;
      VLOG(2) << "selectPort: selecting first input";
      return true;
    }
   
    // loop through inputs of a specific source_type, if any
    
    if (selected_node->inputs.find(source_type) == selected_node->inputs.end()) {
      VLOG(2) << "selectPort: no matching inputs of type " << source_type;
      selected_port_ind = 0;
      return false;
    }

    //selected_node->draw_selected_port = true;

    // start at beginning if uninitialized
    // TBD this means inputs can't be protected
    const string first_port = selected_node->inputs[source_type].begin()->first;

    if ((selected_port == "") || 
        (selected_node->inputs[source_type].find(selected_port) == 
         selected_node->inputs[source_type].end())) {
      selected_port = first_port;
      // TBD double book keeping is ugly
      selected_type = source_type;
      selected_node->selected_type = selected_type;
      selected_node->selected_port = selected_port;
      VLOG(2) << "selectPort: start at beginning because uninitialized";
      selected_port_ind = 0;
      return true;
    } 

    // next look for match with current 
    inputsItemType::iterator it2 = selected_node->inputs[source_type].find(selected_port);
    inputsItemType::iterator it2_end = selected_node->inputs[source_type].end();
    
    if (it2 == it2_end) {
      selected_port = first_port;
      // TBD selected_type used anywhere? Yes Nodes uses it. 
      selected_type = source_type;
      selected_node->selected_type = selected_type;
      selected_node->selected_port = selected_port;
      VLOG(2) << "selectPort: match with end so using first port";
      selected_port_ind = 0;
      return true;
    }

    it2++;
    selected_port_ind++;
    if (it2 == it2_end) {
      selected_port = first_port;
      selected_port_ind = 0;
    }
    // now finally set the port to the next available port
    else selected_port = it2->first;
    
    selected_type = source_type;
    selected_node->selected_type = source_type;
    selected_node->selected_port = selected_port;
    
    VLOG(2) << "selectPort: selected next in iterator";
    return true;
  }// selectPort()

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
      selectPort();
      
      stringstream str;
      if (selected_node) {str << selected_node->name << " : ";
      str << "matching " << source_type << " " << source_port << " with ";
      } else {
        str <<"selecting";
      }
      str  << CLTXT << selected_ind << " " << selected_port_ind << " " 
          << selected_type << " " << selected_port << CLNRM;
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
      float scale = 0.2;
      if (all_nodes[i] == output_node) scale = 0.3;
      if (i == 0) scale = 0.3;
      all_nodes[i]->draw(scale);
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

