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

/*
#include <linux/input.h>
#include <fcntl.h>
*/

#include <iostream>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <time.h>

#include <deque>

#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>

#include <ros/console.h>
#include <std_msgs/String.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "config.h"
#include "nodes.h" 
#include "modify.h" 
#include "misc_nodes.h" 
#include "signals.h"
#include "filter.h"
#include "cluster.h"
#include "generate.h"
//#include "screencap.h"
#include "output.h"
//#include "input.h"
#include "structure.h"
//#include "opengl.h"
#include "video.h"

using namespace cv;
//using namespace std;

//DEFINE_string(graph, "../temp_graph.yml", "yaml file to load with graph in it");


//DEFINE_bool(
namespace bm {

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

class VimJay : public Output
{
  // This probably isn't super low latency, but going to try it out
  ros::Subscriber key_sub;
  std::deque<int> keys;
  // TBD informal timer for the system
  
  // make sure all Nodes are stored here
  std::deque<boost::shared_ptr<Node> > all_nodes;

  // the final output 
  // TBD make this a special node type
  boost::shared_ptr<Output> output_node;
  boost::shared_ptr<Output> preview_node;

  // TBD temp- currently need to connect output Xlib parameters to Mouse
  //boost::shared_ptr<Mouse> input_node;

  // where the ui pointer is (currently controlled by keyboard
  cv::Point cursor;
  // scale factor on drawing ui elements
  float zoom;

  public:
 
  // where the upper left coordinate of the window into the ui is
  cv::Point2f ui_offset;

  /////////////////////////////////////////////////////////////////////////////
  // conveniently create and store node
  template <class nodeType>
    boost::shared_ptr<nodeType> getNode(string name = "", cv::Point2f loc=cv::Point2f(0.0, 0.0))
    {
      boost::shared_ptr<nodeType> node = 
          boost::shared_ptr<nodeType>(new nodeType(name));//name, loc, graph_ui);
      
      node->init();

      ROS_DEBUG_STREAM_COND(log_level > 1, CLVAL << all_nodes.size()  << CLTX2 
          << " new node " << CLNRM << " " << getId(node) << " "  
          << name << " " << loc.x << ", " << loc.y << " " << node);

      node->name = name;
      node->loc = loc;
      node->graph_ui = graph_ui;

      // instead of push_back create a gateway function to handle it
      all_nodes.push_back(node);

      return node;
    }

    /////////////////////////////////////////////////////////////////////////// 
    boost::shared_ptr<Node> getNodeByName(
        std::string type_id, 
        std::string name = "", 
        cv::Point2f loc = cv::Point2f(0.0, 0.0)
        )
    {
      boost::shared_ptr<Node> node = boost::shared_ptr<Node>();

      if (type_id.compare("bm::VimJay") == 0) {
        ROS_INFO_STREAM("already have cam thing");
        /*
        // TBD  don't allow duplicate camthings
        ROS_DEBUG_STREAM_COND(log_level > 1, CLVAL << all_nodes.size()  << CLTX2 
          << " new node " << CLNRM << name << " " << loc.x << ", " << loc.y);
        // the node already exists
        //node = getNode<ScreenCap>(name, loc);
        node = boost::dynamic_pointer_cast<Node>(shared_from_this());
        node->loc = loc;
        node->name = name;
        all_nodes.push_back(node);
        */
      //} else if (type_id.compare("bm::ScreenCap") == 0) {
      //  node = getNode<ScreenCap>(name, loc);
      //  node->update();
      } else if (type_id.compare("bm::Webcam") == 0) {
        node = getNode<Webcam>(name, loc);
      } else if (type_id.compare("bm::Video") == 0) {
        node = getNode<Video>(name, loc);
      } else if (type_id.compare("bm::ImageNode") == 0) {
        node = getNode<ImageNode>(name, loc);
      } else if (type_id.compare("bm::Sobel") == 0) {
        node = getNode<Sobel>(name, loc);
      } else if (type_id.compare("bm::Laplacian") == 0) {
        node = getNode<Laplacian>(name, loc);
      } else if (type_id.compare("bm::PyrMean") == 0) {
        node = getNode<PyrMean>(name, loc);
      } else if (type_id.compare("bm::GaussianBlur") == 0) {
        node = getNode<GaussianBlur>(name, loc);
      } else if (type_id.compare("bm::MedianBlur") == 0) {
        node = getNode<MedianBlur>(name, loc);
      } else if (type_id.compare("bm::BilateralFilter") == 0) {
        node = getNode<BilateralFilter>(name, loc);
      //} else if (type_id.compare("bm::InPaint") == 0) {
      //  node = getNode<InPaint>(name, loc);
      //} else if (type_id.compare("bm::OpenGL") == 0) {
      //  node = getNode<OpenGL>(name, loc);
      } else if (type_id.compare("bm::OpticalFlow") == 0) {
        node = getNode<OpticalFlow>(name, loc);
      } else if (type_id.compare("bm::Buffer") == 0) {
        node = getNode<Buffer>(name, loc);
      } else if (type_id.compare("bm::SigToInd") == 0) {
        node = getNode<SigToInd>(name, loc);
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
      } else if (type_id.compare("bm::BrowseDir") == 0) {
        node = getNode<BrowseDir>(name, loc);
      } else if (type_id.compare("bm::Mean") == 0) {
        node = getNode<Mean>(name, loc);
      } else if (type_id.compare("bm::Add") == 0) {
        node = getNode<Add>(name, loc);
      } else if (type_id.compare("bm::AddMasked") == 0) {
        node = getNode<AddMasked>(name, loc);
      } else if (type_id.compare("bm::Multiply") == 0) {
        node = getNode<Multiply>(name, loc);
      } else if (type_id.compare("bm::AbsDiff") == 0) {
        node = getNode<AbsDiff>(name, loc);
      } else if (type_id.compare("bm::Greater") == 0) {
        node = getNode<Greater>(name, loc);
      } else if (type_id.compare("bm::Max") == 0) {
        node = getNode<Max>(name, loc);
      } else if (type_id.compare("bm::Resize") == 0) {
        node = getNode<Resize>(name, loc);
      } else if (type_id.compare("bm::Flip") == 0) {
        node = getNode<Flip>(name, loc);
      } else if (type_id.compare("bm::EqualizeHist") == 0) {
        node = getNode<EqualizeHist>(name, loc);
      } else if (type_id.compare("bm::Normalize") == 0) {
        node = getNode<Normalize>(name, loc);
      } else if (type_id.compare("bm::Distance") == 0) {
        node = getNode<Distance>(name, loc);
      } else if (type_id.compare("bm::FloodFill") == 0) {
        node = getNode<FloodFill>(name, loc);
      } else if (type_id.compare("bm::MorphologyEx") == 0) {
        node = getNode<MorphologyEx>(name, loc);
      } else if (type_id.compare("bm::Rot2D") == 0) {
        node = getNode<Rot2D>(name, loc);
      } else if (type_id.compare("bm::Undistort") == 0) {
        node = getNode<Undistort>(name, loc);
      } else if (type_id.compare("bm::Remap") == 0) {
        node = getNode<Remap>(name, loc);
      } else if (type_id.compare("bm::Kaleid") == 0) {
        node = getNode<Kaleid>(name, loc);
      } else if (type_id.compare("bm::Signal") == 0) {
        node = getNode<Signal>(name, loc);
      } else if (type_id.compare("bm::SigAdd") == 0) {
        node = getNode<SigAdd>(name, loc);
      } else if (type_id.compare("bm::SigGreater") == 0) {
        node = getNode<SigGreater>(name, loc);
      } else if (type_id.compare("bm::MiscSignal") == 0) {
        node = getNode<MiscSignal>(name, loc);
      } else if (type_id.compare("bm::SigBuffer") == 0) {
        node = getNode<SigBuffer>(name, loc);
      } else if (type_id.compare("bm::SigBufferXY") == 0) {
        node = getNode<SigBufferXY>(name, loc);
      } else if (type_id.compare("bm::Tap") == 0) {
        node = getNode<Tap>(name, loc);
      } else if (type_id.compare("bm::TapInd") == 0) {
        node = getNode<TapInd>(name, loc);
      } else if (type_id.compare("bm::Contour") == 0) {
        node = getNode<Contour>(name, loc);
      } else if (type_id.compare("bm::DistanceFlip") == 0) {
        node = getNode<DistanceFlip>(name, loc);
      } else if (type_id.compare("bm::Bezier") == 0) {
        node = getNode<Bezier>(name, loc);
      } else if (type_id.compare("bm::Circle") == 0) {
        node = getNode<Circle>(name, loc);
      } else if (type_id.compare("bm::Noise") == 0) {
        node = getNode<Noise>(name, loc);
      } else if (type_id.compare("bm::SimplexNoise") == 0) {
        node = getNode<SimplexNoise>(name, loc);
      } else if (type_id.compare("bm::Trig") == 0) {
        node = getNode<Trig>(name, loc);
      } else if (type_id.compare("bm::SigADSR") == 0) {
        node = getNode<SigADSR>(name, loc);
      #if 0
      // TBD need to make this node just subscribe to joy
      } else if (type_id.compare("bm::GamePad") == 0) {
        node = getNode<GamePad>(name, loc);
      } else if (type_id.compare("bm::Mouse") == 0) {
        node = getNode<Mouse>(name, loc);
        
        if (input_node) {
          ROS_WARN_STREAM("TBD multiple mouse nodes");
        }

        input_node = boost::dynamic_pointer_cast<Mouse>(node);
        // TBD need better way to share X11 info- Config probably
        if (output_node) {
          //input_node->display = output_node->display;
          //input_node->win = output_node->win;
          //input_node->opcode = output_node->opcode;
        } else {
          ROS_ERROR_STREAM("output_node should always exist by this point");
        }
      #endif
      } else if (type_id.compare("bm::Output") == 0) {
        ROS_INFO_STREAM("already created output and preview nodes, ignoring " 
            << CLTXT << name << CLNRM);
        //node = getNode<Output>(name, loc);

      } else {
        ROS_WARN_STREAM("unknown node type " << type_id << ", assuming imageNode");
        node = getNode<ImageNode>(name, loc);
      }

      return node;
    }


    /////////////////////////////////////////////////////////////////////////// 
  // delete all the nodes
  bool clearNodes()
  {
    ROS_INFO_STREAM(CLTX2 << "clearing nodes" << CLNRM);
    update_nodes = false;
    node_thread.join();
    //for (int i = 0; i < all_nodes.size(); i++) {
      // TBD need to gather all displays and XCloseDisplay it

    //}

    // this should cause all smart pointers to delete
    // 3 is the magic number of nodes that persist across
    // loading: cam_thing, preview, and output
    all_nodes.resize(3);
       
    // dangling reference to input_node
    //input_node   = boost::shared_ptr<Mouse>();
  }

  void clearAllNodeUpdates() 
  {
    for (int i = 0; i < all_nodes.size(); i++) {
      all_nodes[i]->do_update = false;
      // TBD for asynchronous this fails, but need buffering to make that work anyhow
      //all_nodes[i]->is_dirty = false;
    }
  }

  int getIndFromPointer(boost::shared_ptr<Node> node)
  {
    if (node == NULL) return -1;

    
    for (int i = 0; i < all_nodes.size(); i++) {
      if (all_nodes[i] == node) {
        ROS_DEBUG_STREAM_COND(log_level > 3, node->name << " " << i); 
        return i; 
      }
    }

    return -1;
  }
  
  

  /// save the graph to an output yaml file
  bool saveGraph(const std::string graph_file="graph.yml") 
  {
    boost::mutex::scoped_lock l(update_mutex);

    ROS_INFO_STREAM("saving graph " << graph_file);
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
      ROS_DEBUG_STREAM_COND(log_level > 2, all_nodes[i]->name << " saving " << all_nodes[i]->ports.size() << " inputs");
      for (int j = 0; j < all_nodes[i]->ports.size(); j++) {
          
          boost::shared_ptr<Connector> con = all_nodes[i]->ports[j];

          fs << "{";
          fs << "type" << con->type;
          fs << "name" << con->name;
          // TBD loc
          if (con->type == SIGNAL) {
            //fs << it->first << it->second;  // this is clever but I don't know how to load it
            fs << "value" << con->value;
          }

          std::string src_port = "";
          int src_ind = -1;
          if (con->src.lock()) {
            src_port = con->src.lock()->name;
            // TBD this function is currently why this isn't in the Node::save()
            src_ind = getIndFromPointer(con->src.lock()->parent.lock());  
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
    
    setString("graph_file", graph_file);

    return true;
  }

  // organize all the nodes, for now based on index but later perhaps based on connectivity
  bool gridGraph() 
  {
    paused = true;

    const int wd = (sqrt(all_nodes.size()));
    
    const float tht = Config::inst()->thumb_height;
    
    ROS_INFO_STREAM("making " << all_nodes.size() << " graph items into grid");

    int x = 40;
    int y = 20;
    for (int i = 0; i < all_nodes.size(); i++) {
      
      if ( boost::dynamic_pointer_cast<ImageNode> (all_nodes[i])) y += tht;

      if (y + all_nodes[i]->ports.size()*10 > graph_ui.rows) {
        y = 20;
        if ( boost::dynamic_pointer_cast<ImageNode> (all_nodes[i])) y += tht;

        x += 180;
      }
      cv::Point loc = cv::Point2f( x, y );
      
      ROS_DEBUG_STREAM_COND(log_level > 1, i << " " << getId(all_nodes[i]) << " "
          <<  all_nodes[i]->name << " " 
          << all_nodes[i]->loc.x << " " << all_nodes[i]->loc.y 
          << " -> " << loc.x << " " << loc.y << ", " <<all_nodes[i]->ports.size());
      
      all_nodes[i]->loc = loc;

      y += all_nodes[i]->ports.size()*10 + 60;
      //if ( (ImageNode*) all_nodes[i]) y += tht;
      // TBD camthing can grow with input devices
      if (i == 0) { 
        y += tht;
      }
      
    }
  } // gridGraph

  int count;

  cv::Mat test_im;
  cv::Mat cam_image;

  // TBD make struct for these two
  int selected_ind;
  boost::shared_ptr<Node> selected_node;

  // store a node to be connected to a different selected node
  int source_ind;
  boost::shared_ptr<Node> source_node;
  // ImageNode, Signal, or Buffer for now- TBD use enums instead of strings
  //conType source_type;
  //string source_port;
  //int source_port_ind;
  
  boost::thread node_thread;

  std::vector<string> node_types;

  std::string load_graph_file_;

  /////////////////////////////////////////////////////////////////////////// 
  VimJay(const std::string name) :
      Output(name),
      selected_ind(0), 
      source_ind(0),
      //source_type(NONE),
      //source_port(""),
      //source_port_ind(0),
      draw_nodes(true),
      //paused(true)
      load_graph_file_(""),
      paused(true) // TBD control from config file?
  {
    key_sub = Config::inst()->nh_.subscribe<std_msgs::String>("key", 5,
      &VimJay::keyCallback, this);

    // TBD call init from constructor
    Config::inst()->nh_.getParam("graph_file", load_graph_file_);
    ROS_INFO_STREAM("graph file " << load_graph_file_);
  }

  void keyCallback(const std_msgs::StringConstPtr& msg) 
  { 
    //ROS_INFO_STREAM("keys " << msg->data);
    for (size_t i = 0; i < msg->data.size(); ++i) {
      keys.push_back(msg->data[i]);
    }
  }

  virtual void init()
  { 
    // TBD use a different methodology that doesn't require
    // calling the base function?
    Output::init();

    count = 0;

    //node_types.push_back("bm::VimJay"); // TBD can't spawn a new one of these
    node_types.push_back("bm::ImageDir");
    node_types.push_back("bm::BrowseDir");
    node_types.push_back("bm::Webcam");
    node_types.push_back("bm::Video");
    //node_types.push_back("bm::ScreenCap");
    node_types.push_back("bm::Rot2D");
    node_types.push_back("bm::SigToInd");
    node_types.push_back("bm::Buffer");
    node_types.push_back("bm::Mux");
    node_types.push_back("bm::Laplacian");
    node_types.push_back("bm::Add");
    node_types.push_back("bm::AddMasked");
    node_types.push_back("bm::Multiply");
    node_types.push_back("bm::AbsDiff");
    node_types.push_back("bm::Greater");
    node_types.push_back("bm::Max");
    node_types.push_back("bm::SigAdd");
    node_types.push_back("bm::SigGreater");
    node_types.push_back("bm::Trig");
    
    node_types.push_back("bm::FloodFill");
    
    node_types.push_back("bm::ImageNode");
    node_types.push_back("bm::Sobel");
    node_types.push_back("bm::PyrMean");
    node_types.push_back("bm::GaussianBlur");
    node_types.push_back("bm::MedianBlur");
    node_types.push_back("bm::BilateralFilter");
    //} else if (type_id.compare("bm::InPaint");
    node_types.push_back("bm::OpticalFlow");
    node_types.push_back("bm::MuxBuffer");
    node_types.push_back("bm::FilterFIR");
    node_types.push_back("bm::Cluster");
    node_types.push_back("bm::Mean");
    node_types.push_back("bm::Resize");
    node_types.push_back("bm::Flip");
    node_types.push_back("bm::EqualizeHist");
    node_types.push_back("bm::Normalize");
    node_types.push_back("bm::Distance");
    node_types.push_back("bm::MorphologyEx");
    node_types.push_back("bm::Undistort");
    node_types.push_back("bm::Remap");
    node_types.push_back("bm::Kaleid");
    node_types.push_back("bm::Signal");
    node_types.push_back("bm::MiscSignal");
    node_types.push_back("bm::SigBuffer");
    node_types.push_back("bm::SigBufferXY");
    node_types.push_back("bm::Tap");
    node_types.push_back("bm::TapInd");
    node_types.push_back("bm::Contour");
    node_types.push_back("bm::DistanceFlip");
    node_types.push_back("bm::Bezier");
    node_types.push_back("bm::Circle");
    node_types.push_back("bm::Noise");
    node_types.push_back("bm::SimplexNoise");

    //node_types.push_back("bm::OpenGL");
    
    node_types.push_back("bm::SigADSR");
    //node_types.push_back("bm::GamePad");
    //node_types.push_back("bm::Mouse");
    //node_types.push_back("bm::Output"); // TBD if can spawn this


    setSignal("node_ind", 0, false, ROLL, 0, node_types.size()-1);
    setString("node", node_types[0], true);

    setSignal("x", 0);

    // TBD make internal type a gflag
    // or have a config file that loads it and resolution also
    graph_ui = cv::Mat(cv::Size(Config::inst()->ui_width, Config::inst()->ui_height), MAT_FORMAT_C3);
    graph_ui = cv::Scalar(0);

    
    // camthing always exists, put it in first
    loc = cv::Point2f(400,400);

    {
      boost::shared_ptr<Node> node =
        boost::dynamic_pointer_cast<Node>(shared_from_this());
      // TBD these shouldn't be necessary

      all_nodes.push_back(node);
    }

    //// make default output nodes
    {
      boost::shared_ptr<Node> node1 = getNode<Output>("output", loc);
      output_node = boost::dynamic_pointer_cast<Output>(node1);
      output_node->setup(Config::inst()->out_width, Config::inst()->out_height);
      // force output node to move window
      //output_node->draw(ui_offset);
      //output_node->setSignal("x", Config::inst()->ui_width + 28);
      //output_node->draw(ui_offset);
    }
    
    {
      boost::shared_ptr<Node> node2 = getNode<Output>("preview", loc);
      preview_node = boost::dynamic_pointer_cast<Output>(node2);
      preview_node->setup(Config::inst()->out_width, Config::inst()->out_height);
    }
  
    // TBD
    if ((load_graph_file_ == "") || (!loadGraph(load_graph_file_))) { 
      defaultGraph();
    } 
    output_node->setSignal("force_update", 1.0);
    preview_node->setSignal("force_update", 1.0);

    saveGraph("graph_load_test.yml");

    selectNextNode();

    // Xlib stuff
    {
      setup(graph_ui.size().width, graph_ui.size().height);

#if 0
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
      XISelectEvents(display, DefaultRootWindow(display), &eventmask, 1);
      //XISelectEvents(display, win, &eventmask, 1);
#endif    
    }

    update_nodes = true;
    node_thread = boost::thread(&VimJay::updateThread, this);

  }

  ///////////////////////////////////////////////
  bool loadGraph(const std::string graph_file)
  {
    boost::mutex::scoped_lock l(update_mutex);
    ROS_INFO_STREAM("loading graph " << CLTXT << graph_file << CLNRM);
    
    FileStorage fs; 
    fs.open(graph_file, FileStorage::READ);
    
    if (!fs.isOpened()) {
      ROS_ERROR_STREAM("couldn't open " << graph_file;
      return false);
    }
  
    FileNode nd = fs["nodes"]; 
    if (nd.type() != FileNode::SEQ) {
      ROS_ERROR_STREAM("no nodes");

      return false;
    }

    for (FileNodeIterator it = nd.begin(); it != nd.end(); ++it) {
      std::string type_id = (*it)["typeid"];
      std::string name;
      (*it)["name"] >> name;
      cv::Point loc;
      loc.x = (*it)["loc"][0];
      loc.y = (*it)["loc"][1];
      bool enable;
      (*it)["enable"] >> enable;
      
      boost::shared_ptr<Node> node = getNodeByName(type_id, name, loc);
      
      if (node == NULL) { 
        ROS_ERROR_STREAM("couldn't get node made " << type_id << " " << name);
        continue;
      }

      if ( boost::dynamic_pointer_cast<ImageNode>(node)) {
        ( boost::dynamic_pointer_cast<ImageNode> (node))->setImage("out", test_im);
      }
      node->load(it);
      
      // TBD do nothing special for these
      /*
      if (name == "output") {
        output_node = boost::dynamic_pointer_cast<Output>(node);
        cv::Mat tmp;
        node->setImage("in", tmp); 
      }
      if (name == "preview") {
        preview_node = boost::dynamic_pointer_cast<Output>(node);
      }
      */

      ROS_DEBUG_STREAM_COND(log_level > 1, type_id << " " << CLTXT << name << CLVAL << " " 
          << node  << " " << loc << " " << enable << CLNRM);
      
      int ind;
      (*it)["ind"] >> ind;
      ROS_DEBUG_STREAM_COND(log_level > 1, CLTXT << "first pass inputs " << CLVAL << ind << CLNRM << " " << node->name);

      for (int i = 0; i < (*it)["inputs"].size(); i++) {
        int type;
        std::string port;

        (*it)["inputs"][i]["type"] >> type;
        (*it)["inputs"][i]["name"] >> port;
      
        ROS_DEBUG_STREAM_COND(log_level > 1, "input " << ind << " \"" << node->name
            << "\", type " << type << " " << port);
        
        // TBD make function for this
        /*
        conType con_type = NONE;
        if (type == "ImageNode") con_type = IMAGE;
        if (type == "ImageOut") con_type = IMAGE;
        if (type == "Signal") con_type = SIGNAL;
        if (type == "Buffer") con_type = BUFFER;
        */
        
        node->setInputPort((conType)type, port); //, NULL, "");
      }
    }

    // second pass for inputs (the first pass was necessary to create them 
    // all in right order
    ROS_DEBUG_STREAM_COND(log_level > 1, " ");
    ROS_INFO_STREAM("second pass inputs");
    ROS_DEBUG_STREAM_COND(log_level > 1, " ");
    for (FileNodeIterator it = nd.begin(); it != nd.end(); ++it) {
      int ind;
      (*it)["ind"] >> ind;
      ROS_DEBUG_STREAM_COND(log_level > 2, "second pass inputs " << ind << " " << CLTXT << all_nodes[ind]->name << CLNRM);
      for (int i = 0; i < (*it)["inputs"].size(); i++) {
        int input_ind;
        int type;
        std::string port;
        std::string src_port;
        float value;

        (*it)["inputs"][i]["type"] >> type;
        (*it)["inputs"][i]["name"] >> port;
        (*it)["inputs"][i]["src_ind"] >> input_ind;
        (*it)["inputs"][i]["src_port"] >> src_port;
        (*it)["inputs"][i]["value"] >> value;
        
        if (input_ind >= 0) {
        ROS_DEBUG_STREAM_COND(log_level > 2, "input " 
            << " " << input_ind << ", type " << type << " " << port << " " << input_ind
            << " " << src_port);
      
          all_nodes[ind]->setInputPort((conType)type, port, all_nodes[input_ind], src_port);
        } // input_ind > 0

        if (type == SIGNAL) {
          
          all_nodes[ind]->setSignal(port, value);
        }

      }
    } // second input pass

    if (output_node == NULL) {
      ROS_WARN_STREAM(CLWRN << "No output node found, setting it to " 
            << all_nodes[all_nodes.size() - 1]->name << CLNRM);
      // TBD could make sure that this node is an output node
      
      output_node = boost::dynamic_pointer_cast<Output>( 
          all_nodes[all_nodes.size() - 1]);
    }

    ROS_INFO_STREAM(all_nodes.size() << " nodes total");
    //output_node->loc = cv::Point2f(graph.cols - (test_im.cols/2+100), 20);
   
    setString("graph_file", graph_file);

    return true;

  } // loadGraph

  void defaultGraph() 
  {
    {
    boost::mutex::scoped_lock l(update_mutex);

    ROS_INFO_STREAM("creating default graph");
    
    // create a bunch of nodes of every type (TBD make sure new ones get added)
    // but don't connect them, user will do that
   



    // Images
    // inputs
    //node = getNode<Webcam>("web_cam", loc);
    //node->update();
#if 0
    node = getNode<ScreenCap>("screen_cap", loc);
    //node->update();

    ImageDir* im_dir1 = getNode<ImageDir>("image_dir", loc);
    im_dir1->setString("dir", "../data");
    im_dir1->loadImages();

    ImageDir* im_dir2 = getNode<ImageDir>("image_dir", loc);
    im_dir2->setString("dir", "../data");
    im_dir2->loadImages();

    // process
    node = getNode<Rot2D>("rot2d", loc);
    node = getNode<Undistort>("undistort", loc);
    node = getNode<Remap>("remap", loc);
    node = getNode<Kaleid>("kaleid", loc);
    
    getNode<Buffer>("buffer", loc);
    getNode<Mux>("mux", loc);
    getNode<MuxBuffer>("mux_buffer", loc);
    getNode<FilterFIR>("filter_fir", loc);
    getNode<Cluster>("cluster", loc);

    // basic operations
    node = getNode<Add>("add", loc);
    node = getNode<AddMasked>("add_masked", loc);
    node = getNode<Multiply>("multiply", loc);
    node = getNode<AbsDiff>("abs_diff", loc);
    node = getNode<Greater>("greater", loc);
    node = getNode<Max>("max", loc);
    
    // filters and manipulations
    node = getNode<Sobel>("sobel", loc);
    node = getNode<Laplacian>("laplacian", loc);
    node = getNode<PyrMean>("pyr_mean", loc);
    node = getNode<GaussianBlur>("gauss_blur", loc);
    node = getNode<MedianBlur>("median_blur", loc);
    node = getNode<BilateralFilter>("bilateral_filter", loc);
    //node = getNode<InPaint>("in_paint", loc);
    
    node = getNode<Resize>("resize", loc);
    node = getNode<Flip>("flip", loc);
    node = getNode<MorphologyEx>("morphology_ex", loc);
    node = getNode<OpticalFlow>("optical_flow", loc);
    //node = getNode<OpenGL>("opengl", loc);
    
    // generate
    node = getNode<Bezier>("bezier", loc);
    node = getNode<Circle>("circle", loc);
    node = getNode<Noise>("noise", loc);
    node = getNode<SimplexNoise>("simplex_noise", loc);

    node = getNode<Tap>("tap0", loc);
    node = getNode<TapInd>("tapind0", loc);

    // structure
    node = getNode<Contour>("contour", loc);

    // Signals
    // inputs
    node = getNode<Mouse>("mouse", loc);
    input_node = (Mouse*) node;

    // generate
    node = getNode<SigAdd>("sigadd", loc);
    node = getNode<Signal>("signal0", loc);
    node = getNode<MiscSignal>("saw", loc);
    node = getNode<Trig>("trig", loc);
    
    node = getNode<SigBuffer>("sig_buf", loc);
#endif    
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

      std::vector<float> vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );

      fir->setup(vec);
      fir->setInputPort(IMAGE,"in", passthrough, "out");

      // IIR denominator
      FilterFIR* denom = getNode<FilterFIR>("iir_denom", cv::Point2f(500,350));
      static float arr2[] =  { 
                 1.7600418803, // y[n-1]
                -1.1828932620, // * y[n- 2])
                0.2780599176, // * y[n- 3])
                }; 

      std::vector<float> vec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );
      denom->setup(vec2);
      denom->setInputPort(IMAGE,"in",fir, "out");
       
      Add* add_iir = getNode<Add>("add_iir", cv::Point2f(400,100) );
      add_iir->out = test_im;
      std::vector<boost::shared_ptr<ImageNode>> add_in;
      add_in.push_back(fir);
      add_in.push_back(denom);
      std::vector<float> nf;
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
   
    }

    nodeUpdate();
    gridGraph(); 

    saveGraph("default_graph.yml");
  }

  bool selectSourceNode() 
  {

    if (selected_node && (source_node != selected_node)) {
      // select source node
      source_ind = selected_ind;
      source_node = selected_node; //all_nodes[source_ind];
      //source_port = selected_node->selected_port;
      //source_type = selected_node->selected_type;

      ROS_DEBUG_STREAM_COND(log_level > 1, "selected source node " << source_node->selected_type << " " << source_node->selected_port); 
    } else {
      // select no source port, allows cycling through all inputs
      source_ind = 0;
      source_node = boost::shared_ptr<Node>();
      //source_type = NONE;
      //source_port = "";
      ROS_DEBUG_STREAM_COND(log_level > 1, "cleared source node");
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
        (source_node->selected_type != NONE) && 
        (selected_node->selected_port != "") && 
        (source_node->selected_port != "") && 
        (selected_node->selected_port != "") && 
        (selected_node->selected_port != "out") // TBD delete this
        ) {

        
        //TBD 
        //if (!selected_node->selected_port_internally_set) 
        {
          // TBD a Buffer should act as an ImageNode if that is the only
          // input available
          selected_node->setInputPort(source_node->selected_type, selected_node->selected_port, source_node, source_node->selected_port);
        }

      return true;
    }  // legit source_node
    
    return false;
  }
  
  bool updatePreviewNode()
  {
    if (!preview_node) return false;
    if (!selected_node) return false;

    boost::shared_ptr<ImageNode> im_src = 
        boost::dynamic_pointer_cast<ImageNode> (selected_node);
    if (!im_src) {
      ROS_DEBUG_STREAM_COND(log_level > 1, "no images to preview");
      return false;
    }

    if (selected_node->selected_type == IMAGE) {
      preview_node->setInputPort(IMAGE, "in", selected_node, selected_node->selected_port);
    } else {
      preview_node->setInputPort(IMAGE, "in", selected_node, "out");
    }
    
    return true;
  } // updatePreviewNode

  bool removePortConnection() 
  {
    boost::mutex::scoped_lock l(update_mutex);

    // disconnect current input
    if (selected_node && (selected_node->selected_type != NONE) && (selected_node->selected_port != "")) {
      selected_node->setInputPort(selected_node->selected_type, selected_node->selected_port); //, NULL, "");
    }
  }

  void selectNextNode() 
  {
    //if (selected_node) selected_node->draw_selected_port = false;
    
    // move forward in selection
    selected_ind++;
    if (selected_ind >= all_nodes.size()) selected_ind = 0;
    selected_node = all_nodes[selected_ind];

    selectPort(0);

    ROS_DEBUG_STREAM_COND(log_level > 1, "selected node " 
        << selected_node->name << " " << selected_ind);
  }

  void selectPrevNode()
  {
    //if (selected_node) selected_node->draw_selected_port = false;
    // move backward in selection
    selected_ind--;
    if (selected_ind < 0) selected_ind = all_nodes.size()-1;
    selected_node = all_nodes[selected_ind];

    selectPort(0);
    
    ROS_DEBUG_STREAM_COND(log_level > 1, "selected node " 
        << selected_node->name << " " << selected_ind);
  }

  bool selectPortSource() 
  {
    ROS_DEBUG_STREAM_COND(log_level > 1, "selecting port source");
    // jump to the node and port inputting into this port
    if (!selected_node) return false;
    if (selected_node->selected_port_ind < 0) return false;
    if (selected_node->selected_port_ind >= selected_node->ports.size()) return false;

    boost::shared_ptr<Connector> con = 
        selected_node->ports[selected_node->selected_port_ind];
    if (!con->src.lock()) return false;

    selected_node = con->src.lock()->parent.lock();
    selected_ind = getIndFromPointer(selected_node); 

    selected_node->selected_port_ind = selected_node->getIndFromPointer(con->src.lock());

    // tell the node to select the port
    selected_node->selectPortByInd(selected_node->selected_port_ind);
    // keep a copy of the selected port local (TBD)
    selectPort(0);
    
    ROS_DEBUG_STREAM_COND(log_level > 2, "selecting port source success");
    return true;
  }

  bool selectPortDestination() 
  {
    ROS_DEBUG_STREAM_COND(log_level > 1, "selecting port destination");
    // jump to the node and port inputting into this port
    if (!selected_node) return false;
    if (selected_node->selected_port_ind < 0) return false;
    if (selected_node->selected_port_ind >= selected_node->ports.size()) return false;

    boost::shared_ptr<Connector> con = selected_node->ports[selected_node->selected_port_ind];
    if (!con->dst) return false;
    
    const std::string src = con->parent.lock()->name;

    selected_node = con->dst->parent.lock();
    selected_ind = getIndFromPointer(selected_node); 

    selected_node->selected_port_ind = selected_node->getIndFromPointer(con->dst);

    // tell the node to select the port
    selected_node->selectPortByInd(selected_node->selected_port_ind);
    // keep a copy of the selected port local (TBD)
    selectPort(0);

    const std::string dst = selected_node->name;
    ROS_DEBUG_STREAM_COND(log_level > 2, "selecting port destination success: " 
        << src << " to " << dst);
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
      ROS_DEBUG_STREAM_COND(log_level > 1, "selectPort: no selected_node");
      return false;
    }
    
    selected_node->draw_selected_port = true;

    // take the current port
    if (next_port == 0) {
      updatePreviewNode();
      return true;
    }

    
    conType source_type = NONE;
    if (source_node) source_type = source_node->selected_type;

    bool rv;
    if (next_port > 0) 
      rv = selected_node->getNextPort(source_type);
    if (next_port < 0)
      rv = selected_node->getPrevPort(source_type);
    
    if (rv) {
      updatePreviewNode();
    }

    return rv;
  } // selectPort()

  std::string command_text;
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
        selectPort(0);
        return true;
      }
      return false;
  }
 
  cv::Point2f ui_vel;
  bool autoScroll()
  {
    if (selected_node) {
      const float pad = 30;
      float dx = 0;
      float dy = 0;

      const cv::Point2f true_upper_left = selected_node->upper_left;
          //selected_node->thumb_offset;
      const cv::Point2f true_lower_right = selected_node->upper_left + 
          selected_node->extent;
      
      // determine if part of the currently selected node is offscreen
      // and if so move the screen so that it is not.
      
      // see if node is hanging off the right or bottom
      dx = true_lower_right.x + pad - (graph_ui.cols - ui_offset.x);
      dy = true_lower_right.y + pad - (graph_ui.rows - ui_offset.y); 
     
      // see if node is hanging off left or top
      if (dx < 0) {
        dx = true_upper_left.x - pad + ui_offset.x;
        if (dx > 0) dx = 0;
      }
      if (dy < 0) {
        dy = true_upper_left.y - pad + ui_offset.y;
        if (dy > 0) dy = 0;
      }

      cv::Point2f acc = cv::Point(dx,dy) * -0.1;
      ui_vel += acc;
      ui_offset += ui_vel;
      ui_vel *= 0.8;

      ROS_DEBUG_STREAM_COND(log_level > 5, selected_node->loc.x << " " << selected_node->loc.y << ", "
        << graph_ui.cols << " " << graph_ui.rows << ", "
        << ui_offset.x << " " << ui_offset.y);
    }
    return true;
  } 

  bool handleInput() 
  {
    count++;
    
    if (keys.empty()) return true;
    int key = keys.front();
    keys.pop_front();
   
    std::cout << key << " key " << std::endl;
    
    bool valid_key = true;

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
      ROS_INFO_STREAM("reloading graph file");
      clearNodes();
      if (!loadGraph(getString("graph_file"))) defaultGraph();
      
      // TBD is there a cleaner place to put this?  In a function
      // at least?  Duplicates with init(), but we don't want to
      // call init here
      update_nodes = true;
      node_thread = boost::thread(&VimJay::updateThread, this);
    }
    else if( key == 'w' ) {
      // TBD increment a count so old saves aren't overwritten?
      std::string base_dir = "../graphs/";
      
      time_t t1 = time(NULL);
      std::stringstream file_name;
      // TBD define path to data somewhere to be reused by all
      file_name << base_dir << t1 << "_" << name << ".yml";
      saveGraph(file_name.str());
      saveGraph("../temp_graph.yml");
      setString("graph_out", file_name.str());
    }
    //else if (key == 'a') {
    //  gridGraph();
    //}
    else if (key == 'z') {
      // TBD need someway to disable this, maybe a camthing signal
      //draw_nodes = !draw_nodes;
      if (selected_node) {
        selected_node->setSignal("force_update",  !(selected_node->getBool("force_update")));
      }
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
      if (selected_node.get() == this) {
        ROS_WARN_STREAM("can't duplicate camthing");
      }
      else if (selected_node) {
        ROS_INFO_STREAM(" duplicating selected_node");
        getNodeByName(getId(selected_node), selected_node->name + "_2", 
            selected_node->loc + cv::Point2f(100,10));
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
        std::stringstream str;
        if (selected_node) {
          str << selected_node->name << " : ";
          conType source_type = NONE;
          std::string source_port = "";
          if (source_node) {
            source_type = source_node->selected_type;
            source_port = source_node->selected_port;
          }
          str << "matching " << source_type << " \"" << source_port << "\" with ";
        } else {
          str << "selecting";
        }

        str 
          << selected_node->selected_type << " \"" << selected_node->selected_port << "\" "
          << CLTXT 
          << selected_ind << " " << selected_node->selected_port_ind << " " 
          << CLNRM;
        ROS_DEBUG_STREAM_COND(log_level > 2, str.str());
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
     
    } else if (key == 'i') {  // ui UP
      ui_offset -= cv::Point2f(0,15);
      ROS_DEBUG_STREAM_COND(log_level > 3, "ui_offset " << ui_offset);
    } else if (key == 'u') {  // ui DOWN
      ui_offset += cv::Point2f(0,15); 
      ROS_DEBUG_STREAM_COND(log_level > 3, "ui_offset " << ui_offset);
    } else if (key == 'y') {  // ui LEFT
      ui_offset -= cv::Point2f(15,0); 
      ROS_DEBUG_STREAM_COND(log_level > 3, "ui_offset " << ui_offset);
    } else if (key == 'o') {  // ui RIGHT
      ui_offset += cv::Point2f(15,0); 
      ROS_DEBUG_STREAM_COND(log_level > 3, "ui_offset " << ui_offset);
    
    //else if (key == 'c') {
      // swap selected node input with source node input- TBD this doesn't work since  
      // outputs can be one-to-many
    //}
    
    } else if (key == '1') {  
      
      if ((selected_node) && (selected_node->selected_type == SIGNAL)) { 
        // create generic signal generator feeding into this port
        boost::shared_ptr<Node> node = 
            getNode<MiscSignal>(
                selected_node->name + "_sig", 
                selected_node->loc + cv::Point2f(-150,10));
   
        float val = selected_node->getSignal(selected_node->selected_port);
        node->setSignal("value", val);
        node->setSignal("min", val);
        node->setSignal("max", val + 1);
        //Connector* con;
        //string src_port;
        //node->getInputPort(SIGNAL, "value", con, src_port);

        selected_node->setInputPort(
            SIGNAL, selected_node->selected_port, 
            node, "value");

      } else if ((selected_node) && 
          (selected_node->selected_port == "dir") ||
          (selected_node->selected_port == "graph_file") ||
          (selected_node->selected_port == "file") 
          ) {
        const std::string dir_or_file = selected_node->selected_port;

        // if the port is named dir or TBD other directory/file names create
        // a BrowseDir
        boost::shared_ptr<Node> node = 
            getNode<BrowseDir>(
                dir_or_file + selected_node->name, 
                selected_node->loc + cv::Point2f(-150,10));
   
        const std::string val = selected_node->getString(selected_node->selected_port);
        node->setString(dir_or_file, val);

        selected_node->setInputPort(
            STRING, selected_node->selected_port, 
            node, dir_or_file);

      }
      // TBD if the port is an image create a buffer for it to output to?

      //node = getNode<MiscSignal>(name, loc);
    } else if (key == '3') {  // connect this image to output in
      if ((selected_node) && (selected_node->selected_type == IMAGE) && (output_node)) { 
        output_node->setInputPort(IMAGE, "in", selected_node, selected_node->selected_port);

      } else {
        // TBD do something to ui when keypress doesn't do anything
      }
    } else {
      valid_key = false;
      // see if node can work with the key
      if (selected_node) 
        valid_key = selected_node->handleKey(key);
    }

    if (valid_key) {
      std::stringstream tmp;
      tmp << (char) key;
      //tmp.resize(1);
      //tmp[0] = key;
      command_text.append(tmp.str());
      ROS_DEBUG_STREAM_COND(log_level > 4, tmp.str() << " " << command_text);
    } else if (key >= 0) {
      ROS_INFO_STREAM("unused keypress:" << key << " " << (char)key);
    }

    int max_count = 24;
    
    int wd = Config::inst()->ui_width;

    if (command_text.size() > wd/32) max_count /= 2;
    if (command_text.size() > wd/24) max_count /= 2;
    if (command_text.size() > wd/16) max_count = 1;
    if (count % max_count == 0) {
      if (command_text.size() > 0);
      command_text = command_text.erase(0,1);
    } else {
      //command_text = "";
    }

    while (command_text.size() > wd/12)
      command_text = command_text.erase(0,1);

    ROS_DEBUG_STREAM_COND(log_level > 5, wd << " " << command_text.size());

    return true;
  } // handleInput

  bool update_nodes;
  boost::mutex update_mutex;

  bool nodeUpdate()
  {
    ROS_DEBUG_STREAM_COND(log_level > 3, CLTXT 
        << "=====================================" << CLNRM);
    boost::mutex::scoped_lock l(update_mutex);

    // TBD will behaviour change depending on the arrangement of nodes-
    // certain parts of the graph will update in different orders depending on
    // the update order.
    for (int i = 0; i < all_nodes.size(); i++) {
      // TBD force update selected node.  This may not always be desired behaviour
      // especially when the user has to cross so many other nodes to get to the
      // one they really want.
      if ((all_nodes[i] == selected_node) || (all_nodes[i]->getBool("force_update")))
        all_nodes[i]->setUpdate();
    }
    for (int i = 0; i < all_nodes.size(); i++) {
      if ((all_nodes[i] == selected_node) || (all_nodes[i]->getBool("force_update")))
        all_nodes[i]->update();
    }

    clearAllNodeUpdates();
  }

  bool updateThread() 
  {
    ROS_INFO_STREAM("starting node update thread");
    while (update_nodes) {

      ROS_DEBUG_STREAM_COND(log_level > 4, "");
      if (!output_node) {
        ROS_ERROR_ONCE("no output_node");
        usleep(20000);
      }

      if (!paused) {
        nodeUpdate();
      } else {
        usleep(10000);
      }
      usleep(1000);
    }
    ROS_INFO_STREAM("ending node update thread");
    return true;
  }

#if 1
  virtual bool update() 
  {
    //ImageNode::update();
    Node::update();

    cv::Mat out = getImage("out");
    if (out.empty()) out = cv::Mat(Config::inst()->getImSize(), MAT_FORMAT_C3);
    cv::resize(graph_ui, out,
          out.size(), 0, 0, cv::INTER_NEAREST );
    setImage("out", out);          

    {
      bool is_valid;
      bool is_dirty;
      const int ind = getSignal("node_ind", is_valid, is_dirty);
      // TBD the node_ind signal may not have been updated here
      if (is_dirty) {
        setString("node", node_types[ind]);
        ROS_DEBUG_STREAM_COND(log_level > 9, "node_ind " << ind << " " << node_types[ind] << " " << getString("node")); //node_types[ind]);
      }
    }

    return true;
  }
#endif

  virtual bool draw(cv::Point2f ui_offset) 
  {
    boost::timer t1;
    
    autoScroll();
    
    if (graph_ui.empty()) {
      ROS_ERROR_STREAM("graph_ui empty");
      return false;
    }

    cv::Mat out_node_im = output_node->getImage("out").clone();

    // draw the output in the background, dimmed down
    if (false) { //(!out_node_im.empty()) {
      cv::resize(out_node_im * (draw_nodes ? 0.2 : 1.0),  graph_ui,
          graph_ui.size(), 0, 0, cv::INTER_NEAREST );
    }
    else
      graph_ui = cv::Scalar(0,0,0);
    
    ROS_DEBUG_STREAM_COND(log_level > 4, "bg draw time" << t1.elapsed()); 

    if (draw_nodes) {
      
      // loop through and draw connectors
      for (int i = 0; i < all_nodes.size(); i++) {
        all_nodes[i]->highlight = (all_nodes[i] == selected_node);
        all_nodes[i]->preDraw(ui_offset);
      }

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
        ROS_DEBUG_STREAM_COND(log_level > 5, "command_text " << command_text);
      }

      
      if (source_node) {
        cv::circle(graph_ui, source_node->loc + ui_offset, 23, cv::Scalar(29,51,11), -1);
        cv::circle(graph_ui, source_node->loc + ui_offset, 22, cv::Scalar(229,151,51), -1);
      }
      ROS_DEBUG_STREAM_COND(log_level > 4, "cv draw time" << t1.elapsed()); 

      // draw input and outputs
      /*
         imshow("cam", cam_buf->get());

         cv::Mat out = output->get();
         if (out.data) {
      //imshow("out", out);
      } else {
      ROS_ERROR_STREAM("out no data");
      }*/
      
      // move nodes away from each other
      nodeRepell();
      
      if (!ImageNode::draw(ui_offset)) {
        //if (!Node::draw(ui_offset)) {
        ROS_ERROR_STREAM("something wrong with node drawing");
        return false;
      }

      // now draw node boxes 
      for (int i = 0; i < all_nodes.size(); i++) {
        if (all_nodes[i].get() != this) {
          all_nodes[i]->draw(ui_offset);
        }
      }

      ROS_DEBUG_STREAM_COND(log_level > 4, "node draw time " << t1.elapsed());
    }

    // commenting this out breaks the drawing, not sure why
    setImage("in", graph_ui );
    ROS_DEBUG_STREAM_COND(log_level > 4, "ui draw time" << t1.elapsed()); 
    
    Output::draw(ui_offset);

    ROS_DEBUG_STREAM_COND(log_level > 4, "full draw time" << t1.elapsed()); 
  } // VimJay::draw

  // cause all the nodes to repel each other so the display isn't too 
  // jumbled
  bool nodeRepell() 
  {
      for (int i = 0; i < all_nodes.size(); i++) {
        for (int j = i+1; j < all_nodes.size(); j++) {
          if (all_nodes[i] == all_nodes[j]) {
            ROS_ERROR_STREAM("duplicate nodes in all_nodes " <<
                all_nodes[i]->name << " " << all_nodes[j]->name);
          }
          //if (i == j) continue;
          
          // loc is actually the upper left corner of the
          // node
          const cv::Point2f l1  = all_nodes[i]->upper_left;
          const cv::Point2f l1b = all_nodes[i]->extent;
          const cv::Point2f l1c = l1 + (l1b * 0.5);
          
          const cv::Point2f l2  = all_nodes[j]->upper_left;
          const cv::Point2f l2b = all_nodes[j]->extent;
          const cv::Point2f l2c = l2 + (l2b * 0.5);
          
          const cv::Point2f dxy = l1c - l2c;

          float dist = sqrt(dxy.x*dxy.x + dxy.y*dxy.y);
          float max_dist1 = sqrt(l1b.x*l1b.x + l1b.y*l1b.y);
          const float max_dist2 = sqrt(l2b.x*l2b.x + l2b.y*l2b.y) + 20;
          if (max_dist2 > max_dist1) max_dist1 = max_dist2;
          max_dist1 *= 0.8;
          
          const float min_dist = 20;
          const float acc = 0.4;
          // repell within this zone
          if (dist < max_dist1) {
            // avoid singularities
            if (dist == 0) { 
              ROS_WARN_STREAM(all_nodes[i]->name << " " << i << ", " 
                  << all_nodes[j]->name << " " << j << " " << dist); 
              all_nodes[i]->acc += cv::Point2f(i/100.0,j/100.0); 
              continue;
            }
            ROS_DEBUG_STREAM_COND(log_level > 3, i << " " << j << " " << dist); 
            if (dist < min_dist) dist = min_dist;

            all_nodes[i]->acc += dxy * (acc/dist);
            all_nodes[j]->acc -= dxy * (acc/dist);
          }
        }
      }

  } // node repell

  virtual bool handleKey(int key) 
  {
    bool valid_key = Node::handleKey(key);
    if (valid_key) return true;

    ROS_INFO_STREAM(" camthing handlekey");
    if (key == '[') {
      int ind = getSignal("node_ind");

      ROS_INFO_STREAM("node_ind " << ind);

      setString("node", node_types[ind]);

      ROS_INFO_STREAM("creating node " << node_types[ind]);
      getNodeByName(node_types[ind], 
              node_types[ind], 
              loc + cv::Point2f(150,10));

    } else {
        valid_key = false;
    }
    
    return valid_key;
  } // handleKey

  }; // vimjay

}


/*
 * To work with Kinect the user must install OpenNI library and PrimeSensorModule for OpenNI and
 * configure OpenCV with WITH_OPENNI flag is ON (using CMake).

TBD have a mode that takes a webcam, uses brightness as depth, and thresholds it for the valid map

 */
int main( int argc, char* argv[] )
{
  ros::init(argc, argv, "vimjay");
 
  boost::shared_ptr<bm::VimJay> vimjay(new bm::VimJay("graph"));
  vimjay->init();
  
  float hz = 30;
  ros::param::get("draw_rate", hz);
  ROS_INFO_STREAM("update rate " << hz);
  ros::Rate rate(rate);

  bool rv = true;
  while (rv && ros::ok()) {

    // have to do this before update to avoid some crashes
    // input is handled in the same thread as drawing because of Xlib- TBD
    rv = vimjay->handleInput();
    vimjay->draw(vimjay->ui_offset);
    
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

