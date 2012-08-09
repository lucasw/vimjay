
#include <iostream>
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
#include "filter.h"
#include "screencap.h"

using namespace cv;
using namespace std;

DEFINE_string(graph_file, "../graph.yml", "yaml file to load with graph in it");

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
      
      LOG(INFO) << all_nodes.size() << " new node " << name << " " << loc.x << ", " << loc.y;

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
      
      all_nodes[i]->save(fs);
      fs << "ind" << i; //(int64_t)all_nodes[i];   // 
      fs << "inputs" << "[:";
      for (int j = 0; j < all_nodes[i]->inputs.size(); j++) {
        fs << getIndFromPointer(all_nodes[i]->inputs[j]);   
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

  // store a node to be connected to a different selected node
  int source_ind;
  Node* source_node;
  
  CamThing() : 
      selected_ind(0), 
      selected_node(NULL),
      source_ind(0),
      source_node(NULL),
      output_node(NULL) 
  {
    count = 0;

    // TBD make internal type a gflag
    graph_im = cv::Mat(cv::Size(1280, 720), MAT_FORMAT_C3);
    graph_im = cv::Scalar(0);
 
    loadGraph(FLAGS_graph_file);
    saveGraph("graph_load_test.yml");

    cv::namedWindow("graph_im", CV_GUI_NORMAL);
    cv::moveWindow("graph_im", 0, 500);

/*
    // Bring this back when there is a fullscreen/decoration free output window
    cv::namedWindow("out", CV_GUI_NORMAL);
    cv::moveWindow("out", 420, 0);
*/

  }

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
      LOG(INFO) << type_id << " " << name << " " << loc << " " << enable;
      
      Node* nd;
      if (type_id.compare("bm::Webcam") == 0) {
        // TBD make a version of getNode that takes a type_id string
        Webcam* cam_in = getNode<Webcam>(name, loc);
        nd = cam_in;

        test_im = cv::Mat(cam_in->get().size(), cam_in->get().type());
        test_im = cv::Scalar(200,200,200);
      }
      else if (type_id.compare("bm::ScreenCap") == 0) {
        nd = getNode<ScreenCap>(name, loc);
        nd->update();
      }
      else if (type_id.compare("bm::ImageNode") == 0) {
        nd = getNode<ImageNode>(name, loc);
      }
      else if (type_id.compare("bm::Sobel") == 0) {
        nd = getNode<Sobel>(name, loc);
      }
      else if (type_id.compare("bm::Buffer") == 0) {
        nd = getNode<Buffer>(name, loc);
      }
      else if (type_id.compare("bm::ImageDir") == 0) {
        nd = getNode<ImageDir>(name, loc);
      }
      else if (type_id.compare("bm::Add") == 0) {
        nd = getNode<Add>(name, loc);
      }
      else if (type_id.compare("bm::Rot2D") == 0) {
        nd = getNode<Rot2D>(name, loc);
      }
      else if (type_id.compare("bm::Signal") == 0) {
        nd = getNode<Signal>(name, loc);
      }
      else if (type_id.compare("bm::Saw") == 0) {
        nd = getNode<Saw>(name, loc);
      }
      else if (type_id.compare("bm::Tap") == 0) {
        nd = getNode<Tap>(name, loc);
      }
      else if (type_id.compare("bm::Output") == 0) {
        nd = getNode<Output>(name, loc);
        output_node = (Output*)nd;
      } else{
        LOG(WARNING) << "unknown node type " << type_id << ", assuming imageNode";
        nd = getNode<ImageNode>(name, loc);
      }

      if (dynamic_cast<ImageNode*>(nd)) {
        (dynamic_cast<ImageNode*> (nd))->out = test_im;
      }
      nd->load(it);

    }

    // second pass for inputs
    for (FileNodeIterator it = nd.begin(); it != nd.end(); ++it) {
      int ind;
      (*it)["ind"] >> ind;
      for (int i = 0; i < (*it)["inputs"].size(); i++) {
        int input_ind;
        (*it)["inputs"][i] >> input_ind;
        all_nodes[ind]->inputs.push_back(all_nodes[input_ind]);
      }
    } // second input pass


    if (output_node == NULL) {
      LOG(WARNING) << "No output node found, setting it to " << all_nodes[all_nodes.size() - 1]->name;
      // TBD could make sure that this node is an output node
      output_node = (Output*) all_nodes[all_nodes.size() - 1];
    }

    LOG(INFO) << all_nodes.size() << " nodes total";
    //output_node->loc = cv::Point(graph.cols - (test_im.cols/2+100), 20);
    

  } // loadGraph

  void defaultGraph() 
  {
    ///////////////
    const float advance = 0.1;

    cam_in = getNode<Webcam>("webcam", cv::Point(50, 20) );
    test_im = cv::Mat(cam_in->get().size(), cam_in->get().type());
    test_im = cv::Scalar(200,200,200);

    ImageNode* passthrough = getNode<ImageNode>("image_node_passthrough", cv::Point(400, 50) );
    passthrough->inputs.push_back(cam_in);
    passthrough->out = test_im;
    passthrough->out_old = test_im;
    //output = passthrough;

    Add* add_loop = getNode<Add>("add_loop", cv::Point(600,100) );
    add_loop->out = test_im;

    Rot2D* rotate = getNode<Rot2D>("rotation", cv::Point(400,400));
    rotate->inputs.push_back(add_loop);
    rotate->out = test_im;
    rotate->out_old = test_im;
    rotate->angle = 50.0;
    rotate->center = cv::Point2f(test_im.cols/2, test_im.rows/2);
    //output = rotate;

    Signal* sr = getNode<Saw>("saw_rotate", cv::Point(350,400) ); 
    sr->setup(0.02, -5.0, -5.0, 5.0);
    rotate->inputs.push_back(sr);

    Signal* scx = getNode<Saw>("saw_center_x", cv::Point(350,450) ); 
    scx->setup(5, test_im.cols/2, 0, test_im.cols);
    rotate->inputs.push_back(scx);
    
    Signal* scy = getNode<Saw>("saw_center_y", cv::Point(350,500) ); 
    //scy->setup(6, test_im.rows/2, test_im.rows/2 - 55.0, test_im.rows/2 + 55.0);
    scy->setup(6, test_im.rows/2, 0, test_im.rows);
    rotate->inputs.push_back(scy);

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
      fir->inputs.push_back(passthrough);

      // IIR denominator
      FilterFIR* denom = getNode<FilterFIR>("iir_denom", cv::Point(500,350));
      static float arr2[] =  { 
                 1.7600418803, // y[n-1]
                -1.1828932620, // * y[n- 2])
                0.2780599176, // * y[n- 3])
                }; 

      vector<float> vec2 (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );
      denom->setup(vec2);
      denom->inputs.push_back(fir);
       
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

    saveGraph();
  }
   
  char key;
  bool valid_key;

  bool handleInput() 
  {
    key = waitKey(20);

    valid_key = true;
    if( key == 'q' ) {
      return false;
    }
    // TBD look for /, then make next letters type search for nodes with names container the following string
    
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
    else if (key == 'g') {
      gridGraph();
    }
    else if (key == 's') {
      if (selected_node) selected_node->enable = !selected_node->enable;
    }
    
    else if (key == 'j') {
      // move forward in selection
      selected_ind++;
      if (selected_ind >= all_nodes.size()) selected_ind = 0;
      selected_node = all_nodes[selected_ind];
    }
    else if (key == 'k') {
      // move backward in selection
      selected_ind--;
      if (selected_ind < 0) selected_ind = all_nodes.size()-1;
      selected_node = all_nodes[selected_ind];
    }
    else if (key == 'r') {
      // select source node
      source_ind = selected_ind;
      source_node = all_nodes[source_ind];
    } 
    else if (key == 't') {
      // select the target node
      // connect to source node in best way possible, replacing the current input
      // TBD need to be able to select specific inputs
      if (source_node) {
      Node* target = all_nodes[selected_ind];

      bool src_image = false;
      bool src_sig = false;

      if (dynamic_cast<ImageNode*> (source_node)) src_image = true;
      if (dynamic_cast<Signal*> (source_node)) src_sig = true;

      if (target->inputs.size() == 0) { 
        target->inputs.push_back(source_node); // maybe will work, TBD need to have more preset input slots
      } else {
        for (int i = 0; i < target->inputs.size(); i++) { 
          if (src_image && (dynamic_cast<ImageNode*>(target->inputs[i]))) {
            target->inputs[i] = (source_node);
            break;
          }
          if (src_sig && (dynamic_cast<Signal*>(target->inputs[i]))) {
            target->inputs[i] = (source_node);
            break;
          }
        }
      }

      }  // legit source_node
    } // set source_node to target input
    else {
      valid_key = false;
    }

    if (valid_key) {
      string tmp;
      tmp.resize(1);
      tmp[0] = key;
      command_text.append(tmp);
    }

    if (count %= 100) {
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
        VLOG(3) << "";
        output_node->setUpdate();
        output_node->update();
      }

    return true;
  }
 
  string command_text;

  void draw() 
  {
    graph_im = cv::Scalar(0,0,0);
  
    cv::putText(graph_im, command_text, cv::Point(10, graph_im.rows-40), 1, 1, cv::Scalar(200,205,195));

    // TBD could all_nodes size have
    if (selected_node) cv::circle(graph_im, selected_node->loc, 18, cv::Scalar(0,220,1), -1);
    if (source_node) cv::circle(graph_im, source_node->loc, 10, cv::Scalar(129,181,51), -1);
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

