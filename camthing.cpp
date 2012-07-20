
#include <iostream>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>


#include <deque>
//#include <pair>

using namespace cv;
using namespace std;

class Signal
{
  public:
  Signal(const float new_step=0.01, const float offset=0.0)
  {
    value = offset;
    step = new_step;
    LOG(INFO) << "Signal " << value << " " << new_step;
  }
  
  virtual void update()
  {
    value += step;
    if (value > 1.0) value = 0.0;
    if (value < 0.0) value = 1.0;
  }
  
  float value;
  float step;
};

class Saw : public Signal
{
  public:
  Saw(const float new_step=0.01, const float offset=0.0) : Signal(new_step, offset) {}

  virtual void update()
  {
    value += step;
    if (value > 1.0) {
      step = -abs(step);
      value = 1.0;
    }
    if (value < 0.0) {
      step = abs(step);
      value = 0.0;
    }
    //LOG(INFO) << step << " " << value;
  }
};

class Buffer
{

  deque<cv::Mat> frames;
  int max_size;
  
  public:
  Buffer(const int max_size=180) {
    this->max_size = max_size;
    LOG(INFO) << "new buffer max_size " << this->max_size;
  }

  void add(cv::Mat new_frame)
  {
    frames.push_back(new_frame);

    while (frames.size() >= max_size) frames.pop_front();
  }

  cv::Mat get(const float fr) 
  {
    if (frames.size() < 1) {
      VLOG(1) << "no frames returning gray";
      cv::Mat tmp = cv::Mat(640, 480, CV_8UC3);
      tmp = cv::Scalar(128);
      return tmp;
    }
    int ind = (int)(fr*(float)frames.size());
    if (fr < 0) {
      ind = frames.size() - ind;
    }
    
    if (ind > frames.size() -1) ind = frames.size()-1;
    if (ind < 0) ind = 0;
    //ind %= frames.size();
   
    //VLOG_EVERY_N(1,10) 
    //LOG_EVERY_N(INFO, 10) << ind << " " << frames.size();
    return frames[ind];
  }


};

class Patch
{
  public:

  Signal* signal;
  Buffer* buffer;

  Patch(Signal* new_signal =NULL, Buffer* new_buffer=NULL)
  {
    signal = new_signal;
    buffer = new_buffer;
  }

  virtual void update()
  {
    if (signal != NULL)
      signal->update();
  }

  // this is sort of strange, maybe should have another object that can be many to one with the buffer?
  virtual cv::Mat get()
  {
    cv::Mat rv = buffer->get(signal->value);
    
    //if (rv.empty())
    if (VLOG_IS_ON(2)) {
    cv::line(rv, cv::Point(0,0), cv::Point( rv.cols, 0), cv::Scalar(0,0,0), 2);
    cv::line(rv, cv::Point(0,0), cv::Point( signal->value* rv.cols, 0), cv::Scalar(255,0,0), 2);
    }

    return rv;
  }
};

class Add : public Patch
{
  public:
  
  // TBD make a vector?
  Patch* p1;
  float f1;

  Patch* p2;
  float f2;

  Add(Patch* np1, Patch* np2, float nf1= 0.5, float nf2 = 0.5)
  {
    p1 = np1;
    p2 = np2;
    f1 = nf1;
    f2 = nf2;
  }

  virtual void update()
  {
    // TBD a patch might get updated many times in one step if it is connected to many places
    p1->update();
    p2->update();
  }

  virtual cv::Mat get()
  { 
    // TBD enforce size sameness
    return p1->get() * f1 + p2->get() * f2;
  }
};

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

  LOG(INFO) << "camera opening ...";
  VideoCapture capture(0); //CV_CAP_OPENNI );
  LOG(INFO) << "done.";

  int count = 0;

  if( !capture.isOpened() )
  {
    LOG(ERROR) << "Can not open a capture object.";
    return -1;
  }
  bool rv1 = capture.set( CV_CAP_PROP_FRAME_WIDTH, 800);
  bool rv2 = capture.set( CV_CAP_PROP_FRAME_HEIGHT, 600);
  LOG(INFO) << "set res " << rv1 << " " << rv2;

  
  const float advance = 0.2;
  
  Buffer* cam_buf = new Buffer(1.0/advance*5);
  
  Signal* s1 = new Saw(advance);
  Patch* p1 = new Patch(s1, cam_buf);
  
  // make a chain
  for (float ifr = advance; ifr <= 1.0; ifr += advance ) {

    Signal* s2 = new Saw(advance, ifr);
    Patch* p2 = new Patch(s2, cam_buf);
    Add* add = new Add(p1, p2, 2.0, -1.0);

    p1 = add;
  }
  

  cv::namedWindow("cam", CV_GUI_NORMAL);
  cv::moveWindow("cam",0,0);
  cv::namedWindow("out", CV_GUI_NORMAL);
  cv::moveWindow("out",640,0);
 
  // get the first black frames out
  capture.grab();
  capture.grab();

  for(;;)
  {

    if( !capture.grab() )
    {
      cout << "Can not grab images." << endl;
      continue;
      //return -1;
    }
    
    {
      cv::Mat frame;
      capture.retrieve(frame); 
      if (frame.empty()) {
        cout << "bad capture" << endl;
        continue;
      }
      imshow("cam",frame);
      // I think opencv is reusing a mat within capture so have to clone it
      cam_buf->add(frame.clone());

      // TBD put this in different thread 
      {
        p1->update();
        imshow("out", p1->get());
      }
    }

    if( waitKey( 10 ) == 'q' )
      break;
  }

  return 0;
}
