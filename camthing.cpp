
#include <iostream>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>


#include <deque>
//#include <pair>

#include "nodes.h" 

using namespace cv;
using namespace std;

namespace bm {

class CamThing
{
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

  VideoCapture capture; //CV_CAP_OPENNI );
  Buffer* cam_buf;  
  int count;

  cv::Mat test_im;

  CamThing() 
  {
    LOG(INFO) << "camera opening ...";
    capture = VideoCapture(0); //CV_CAP_OPENNI );
    LOG(INFO) << "done.";

    count = 0;

    if( !capture.isOpened() )
    {
      LOG(ERROR) << "Can not open a capture object.";
      return;// -1;
    }
    
    bool rv1 = capture.set( CV_CAP_PROP_FRAME_WIDTH, 800);
    bool rv2 = capture.set( CV_CAP_PROP_FRAME_HEIGHT, 600);
    LOG(INFO) << "set res " << rv1 << " " << rv2;

    graph = cv::Mat(cv::Size(1280, 720), CV_8UC3);
    graph = cv::Scalar(0);

    // get the first black frames out
    capture.grab();
    capture.retrieve(test_im); 
    capture.grab();
    
    ///////////////
    const float advance = 0.2;

    cam_buf = getNode<Buffer>("webcam", cv::Point(100,100) );  
    cam_buf->max_size = (1.0/advance*5);

    Signal* s1 = getNode<Saw>("saw", cv::Point(200,50) ); 
    s1->setup(advance, 0);

    Tap* p1 = getNode<Tap>("tap", cv::Point(200,100) );
    //static_cast<Tap*>
    p1->setup(s1, cam_buf);
    p1->out = test_im;

    Add* add_loop = getNode<Add>("add_loop", cv::Point(400,100) );
    add_loop->out = test_im;
    ImageNode* nd = add_loop; 
  
  #if 1
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

    add_loop->setup(nd, p1, 0.95, 0.05);

    LOG(INFO) << all_nodes.size() << " nodes total";

    output = nd;

    cv::namedWindow("cam", CV_GUI_NORMAL);
    cv::moveWindow("cam", 0, 0);
    
    cv::namedWindow("graph", CV_GUI_NORMAL);
    cv::moveWindow("graph", 0, 500);
    
    cv::namedWindow("out", CV_GUI_NORMAL);
    cv::moveWindow("out", 420, 0);

    do_capture = true;
  }

  cv::Mat cam_image;
  cv::Mat graph;
  bool do_capture;

  bool update() {
    count++;

    if (do_capture) {
    if( !capture.grab() )
    {
      cout << "Can not grab images." << endl;
      return true;
    }
    
    {
      capture.retrieve(cam_image); 
      if (cam_image.empty()) {
        cout << "bad capture" << endl;
        return true;
      }
      // I think opencv is reusing a mat within capture so have to clone it
      cam_buf->add(cam_image.clone());

    }
    }

    // TBD put this in different thread 
      {
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
    imshow("cam", cam_buf->get());

    cv::Mat out = output->get();
    if (out.data) {
      imshow("out", out);
    } else {
      LOG(ERROR) << "out no data";
    }

    // loop through
    for (int i = 0; i < all_nodes.size(); i++) {
      all_nodes[i]->draw();
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

