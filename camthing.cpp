
#include <iostream>
#include <stdio.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>


#include <deque>
//#include <pair>

using namespace cv;
using namespace std;

/*
 * To work with Kinect the user must install OpenNI library and PrimeSensorModule for OpenNI and
 * configure OpenCV with WITH_OPENNI flag is ON (using CMake).

TBD have a mode that takes a webcam, uses brightness as depth, and thresholds it for the valid map

 */
int main( int argc, char* argv[] )
{
  google::InitGoogleLogging(argv[0]);
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
    

  // Print some avalible Kinect settings.
  LOG(INFO) << "\nDepth generator output mode:" << endl <<
    "FRAME_WIDTH    " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
    "FRAME_HEIGHT   " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
    "FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;

  cv::namedWindow("frame", CV_GUI_NORMAL);
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
      imshow("frame",frame);
    }

    if( waitKey( 1 ) == 'q' )
      break;
  }

  return 0;
}
