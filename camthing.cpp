
#include <iostream>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>


#include <deque>
//#include <pair>

#include "nodes.h" 
#include "filter.h"

using namespace cv;
using namespace std;

namespace bm {

class CamThing
{
  // TBD informal timer for the system
  
  // make sure all Nodes are stored here
  deque<Node*> all_nodes;

  // the final output 
  // TBD make this a special node type
  ImageNode* output;

  public:

  // conveniently create and store node
  template <class nodeType>
    nodeType* getNode(string name = "", cv::Point loc=cv::Point(0.0, 0.0))
    {
      nodeType* node = new nodeType();

      node->name = name;
      node->loc = loc;
      node->graph = graph;

      LOG(INFO) << "new node " << name << " " << loc.x << ", " << loc.y;

      all_nodes.push_back(node);
      return node;
    }

  void clearAllNodeUpdates() 
  {
    for (int i = 0; i < all_nodes.size(); i++) {
      all_nodes[i]->do_update = false;
      // TBD for asynchronous this fails, but need buffering to make that work anyhow
      all_nodes[i]->is_dirty = false;
    }
  }

  Webcam* cam_in;
  int count;

  cv::Mat test_im;

  cv::Mat cam_image;
  cv::Mat graph;
  bool do_capture;
  
  CamThing() 
  {
    count = 0;

    graph = cv::Mat(cv::Size(1280, 720), CV_32FC3);
    graph = cv::Scalar(0);

    ///////////////
    const float advance = 0.1;

    cam_in = getNode<Webcam>("webcam", cv::Point(50, 20) );
    test_im = cv::Mat(cam_in->get().size(), cam_in->get().type());
    test_im = cv::Scalar(200,200,200);

    ImageNode* passthrough = getNode<ImageNode>("image_node_passthrough", cv::Point(400, 50) );
    passthrough->inputs.push_back(cam_in);
    passthrough->out = test_im;
    passthrough->out_old = test_im;
    output = passthrough;

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
    } else {
      FilterFIR* fir = getNode<FilterFIR>("fir_filter", cv::Point(500,50));

      // http://www-users.cs.york.ac.uk/~fisher/cgi-bin/mkfscript
      static float arr[] = //{ 0.85, -0.35, 0.55, -0.05, };
        {
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
          };
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

      output = fir;
    }
/*  
    Add* add_loop = getNode<Add>("add_loop", cv::Point(400,100) );
    add_loop->out = test_im;
    ImageNode* nd = add_loop; 
  */
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

   // add_loop->setup(nd, p1, 0.95, 0.05);

    LOG(INFO) << all_nodes.size() << " nodes total";

    //output = nd;
    //output = p1;
/*
    cv::namedWindow("cam", CV_GUI_NORMAL);
    cv::moveWindow("cam", 0, 0);
*/

    output->loc = cv::Point(graph.cols - (test_im.cols/2+100), 20);
    
    cv::namedWindow("graph", CV_GUI_NORMAL);
    cv::moveWindow("graph", 0, 500);

/*
    // Bring this back when there is a fullscreen/decoration free output window
    cv::namedWindow("out", CV_GUI_NORMAL);
    cv::moveWindow("out", 420, 0);
*/
    do_capture = true;
  }


  bool update() {
    count++;
    
    // TBD put this in different thread 
      { 
        VLOG(3) << "";
        output->setUpdate();
        output->update();
      }
    
    const char key = waitKey(20);
    if( key == 'q' )
      return false;
    if (key == 's') 
      do_capture = !do_capture;


    return true;
  }
  
  void draw() 
  {
    graph = cv::Scalar(0,0,0);
   
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
      float scale = 0.125;
      if (all_nodes[i] == output) scale = 0.5;
      if (all_nodes[i] == cam_in) scale = 0.5;
      all_nodes[i]->draw(scale);
    } 

    imshow("graph", graph);
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
  // pair of rgb images and depths put together
  
  bm::CamThing* cam_thing = new bm::CamThing();
  
  bool rv = true;
  while(rv) {
    rv = cam_thing->update();
    cam_thing->draw();
    cam_thing->clearAllNodeUpdates();
  }

  return 0;
}

