
#include <iostream>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>


#include <deque>
//#include <pair>

using namespace cv;
using namespace std;

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
      cv::Mat tmp = cv::Mat(640,480,CV_8UC3);
      tmp = cv::Scalar(128);
      return tmp;
    }
    int ind = (int)(fr*(float)frames.size());
    if (fr < 0) {
      ind = frames.size() - ind;
    }
    
    ind %= frames.size();
   
    //VLOG_EVERY_N(1,10) 
    LOG_EVERY_N(INFO,10) << ind << " " << frames.size();
    return frames[ind];
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
  VideoCapture capture( 0); //CV_CAP_OPENNI );
  LOG(INFO) << "done.";

  int count = 0;

  if( !capture.isOpened() )
  {
    LOG(ERROR) << "Can not open a capture object.";
    return -1;
  }

  Buffer* cam_buf = new Buffer();

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
      imshow("out",cam_buf->get(0.0));
    }

    if( waitKey( 1 ) == 'q' )
      break;
  }

  return 0;
}
