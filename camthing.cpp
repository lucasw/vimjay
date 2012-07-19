#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>

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

  // pair of rgb images and depths put together
  deque<depthCam> rgb_depths;

  cout << "Kinect opening ..." << endl;
  VideoCapture capture( CV_CAP_OPENNI );
  cout << "done." << endl;

  int count = 0;

  bool using_kinect = true;
  if( !capture.isOpened() )
  {
    cout << "Can not open a capture object." << endl;
    using_kinect = false;

    
    //if (!capture.isOpened()) {
    
     capture.open(0);
    //}
    if (!capture.isOpened()) {
      cout << "can't open webcam" << endl;
      return -1;
    }
    cout << "opened standard webcam instead" << endl;
  }

  if (using_kinect) {
  if( isSetSXGA )
    capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_SXGA_15HZ );
  else
    capture.set( CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE, CV_CAP_OPENNI_VGA_30HZ ); // default

  // Print some avalible Kinect settings.
  cout << "\nDepth generator output mode:" << endl <<
    "FRAME_WIDTH    " << capture.get( CV_CAP_PROP_FRAME_WIDTH ) << endl <<
    "FRAME_HEIGHT   " << capture.get( CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
    "FRAME_MAX_DEPTH    " << capture.get( CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH ) << " mm" << endl <<
    "FPS    " << capture.get( CV_CAP_PROP_FPS ) << endl;

  cout << "\nImage generator output mode:" << endl <<
    "FRAME_WIDTH    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_WIDTH ) << endl <<
    "FRAME_HEIGHT   " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FRAME_HEIGHT ) << endl <<
    "FPS    " << capture.get( CV_CAP_OPENNI_IMAGE_GENERATOR+CV_CAP_PROP_FPS ) << endl;
  }

  for(;;)
  {

    if( !capture.grab() )
    {
      cout << "Can not grab images." << endl;
      //continue;
      return -1;
    }
    else
    {
      depthCam new_data;

      Mat disparityMap;
      Mat grayImage;
      bool cap_all = true;

      if (using_kinect) {
      if( capture.retrieve( new_data.depth, CV_CAP_OPENNI_DEPTH_MAP ) )
      {
      } else cap_all = false;

      if( retrievedImageFlags[1] && capture.retrieve( disparityMap, CV_CAP_OPENNI_DISPARITY_MAP ) )
      {
        if( isColorizeDisp )
        {
          Mat colorDisparityMap;
          colorizeDisparity( disparityMap, colorDisparityMap, isFixedMaxDisp ? getMaxDisparity(capture) : -1 );
          Mat validColorDisparityMap;
          colorDisparityMap.copyTo( validColorDisparityMap, disparityMap != 0 );
          imshow( "colorized disparity map", validColorDisparityMap );
        }
        else
        {
          imshow( "original disparity map", disparityMap );
        }
      }

      if( capture.retrieve( new_data.valid, CV_CAP_OPENNI_VALID_DEPTH_MASK ) ) {
        imshow( "valid depth mask", new_data.valid );
      } else cap_all = false;

      if( capture.retrieve( new_data.bgr, CV_CAP_OPENNI_BGR_IMAGE ) ) {

        //char str[50];
        //sprintf(str, "kbgr_%d.png", count+100000);
        //imwrite(str, bgrImage);
        imshow( "rgb image", new_data.bgr );
      } else cap_all = false;

      if( retrievedImageFlags[4] && capture.retrieve( grayImage, CV_CAP_OPENNI_GRAY_IMAGE ) )
        imshow( "gray image", grayImage );
      
      } else {
        ////// Non kinect webcam ////////////////////////////////
        
        //capture >> new_data.bgr;

        capture.retrieve(new_data.bgr); 
        if (new_data.bgr.empty()) {
          cout << "bad capture" << endl;
          continue;
        }

        cvtColor(new_data.bgr, new_data.depth, CV_BGR2GRAY);
       
        bool bright_is_distant = false;
        if (bright_is_distant)
          new_data.depth = 255 - new_data.depth;

        new_data.valid = new_data.depth.clone();
        
        if (bright_is_distant)
          threshold(new_data.valid, new_data.valid, 200, 255, cv::THRESH_BINARY_INV);
        else
          threshold(new_data.valid, new_data.valid, 50, 255, cv::THRESH_BINARY_INV);

        imshow("temp", new_data.valid);
        
        new_data.depth.convertTo( new_data.depth, CV_16UC1, 32); // 255 );
        cap_all = true;
      }

      if (cap_all) {
        count++;

        const int buffer_sz = 100;

        {
          // turn invalid parts white to make them maximally distant
          Mat valid16;
          threshold(new_data.valid, valid16, 0, 255, cv::THRESH_BINARY_INV);
          valid16.convertTo( valid16, CV_16UC1, 255 );

          new_data.depth = new_data.depth + valid16;//.clone();

          const float scaleFactor = 0.05f;
          Mat show;

          new_data.depth.convertTo( show, CV_8UC1, scaleFactor );
          imshow( "depth map", show );

        }

        // save only 30 seconds in buffer
        if (rgb_depths.size() < buffer_sz) {
          new_data.bgr = new_data.bgr.clone();
          new_data.depth = new_data.depth.clone();
          new_data.valid = new_data.valid.clone();
          rgb_depths.push_back(new_data);
          if (rgb_depths.size() == buffer_sz) cout << "filled buffer" << endl;
          //rgb_depths.pop_front();
        }


        {
          int ind = count % rgb_depths.size();
          //cout << ind << rgb_depths.size() << endl;

          const float scaleFactor = 0.05f;
          Mat old_map =  rgb_depths[ind].depth;
          Mat old_bgr = rgb_depths[ind].bgr;
          Mat old_valid = rgb_depths[ind].valid;

          // the closer parts of the new image will be in black
          Mat diff = new_data.depth - old_map; 

          Mat diff8;
          diff.convertTo( diff8, CV_8UC1, scaleFactor*10.0 );
          imshow( "diff map", diff8 );

          // the closer part of the old image in white
          cv::Mat depth_mask;
          cv::threshold(diff8, depth_mask, 0, 1, cv::THRESH_BINARY);

          // the closer part of the new image in white
          cv::Mat depth_mask_inv;
          cv::threshold(diff8, depth_mask_inv, 0, 1, cv::THRESH_BINARY_INV);

          std::vector<cv::Mat> d_inv;
          std::vector<cv::Mat> d;

          //depth_mask_inv = depth_mask_inv.mul(new_data.valid);
          //depth_mask = depth_mask.mul(old_valid);

          d_inv.push_back(depth_mask_inv);
          d_inv.push_back(depth_mask_inv);
          d_inv.push_back(depth_mask_inv);
          d.push_back(depth_mask);
          d.push_back(depth_mask);
          d.push_back(depth_mask);

          cv::merge(d_inv, depth_mask_inv);
          cv::merge(d, depth_mask);

          Mat dst = new_data.bgr.mul(depth_mask_inv) + old_bgr.mul(depth_mask);

          imshow("combined", dst); 
        }
      } else {
        cout << "didn't cap all" << endl;
      }
    }

    if( waitKey( 30 ) >= 0 )
      break;
  }

  return 0;
}
