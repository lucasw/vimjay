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

#include "misc_nodes.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <glog/logging.h>

#include <deque>

#include "config.h"

using namespace cv;
using namespace std;

namespace bm {
//////////////////////////////////////////////////

  Rot2D::Rot2D(const std::string name) : ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in",tmp);
    setSignal("phi", 0);
    setSignal("theta", 0);
    setSignal("psi", 0);
    setSignal("z", 573, false, SATURATE, 1, 1e6); // TBD this number
    setSignal("scale", 1.0);
    setSignal("center_x",  Config::inst()->im_width/2 );
    setSignal("center_y",  Config::inst()->im_height/2 );
    setSignal("center_z", 0); // -Config::inst()->im_height/2 );
    setSignal("off_x", Config::inst()->im_width/2 );
    setSignal("off_y", Config::inst()->im_height/2 );
    setSignal("off_z", 0); 
    setSignal("border", 0, false, ROLL, 0, 4);
    setSignal("mode", 0, false, ROLL, 0, 4);
    setSignal("manual_xy", 0.0);

    setSignal("x0", 0 );
    setSignal("x1", 0 );
    setSignal("x2", 0 );
    setSignal("x3", 0 );
  }

  bool Rot2D::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this,40)) return true;

    bool im_dirty;
    cv::Mat in;
    in = getImage("in");
    if (in.empty()) return false;

    // TBD provide normalized, radians, and angle input select
    // euler angles
    float phi   = getSignal("phi")   * M_PI/180.0;    
    float theta = getSignal("theta") * M_PI/180.0;    
    float psi   = getSignal("psi")   * M_PI/180.0;   

    float scale = getSignal("scale");    

    cv::Point3f center;
    center.x = getSignal("center_x");     
    center.y = getSignal("center_y");
    center.z = getSignal("center_z");
    
    const float off_x = getSignal("off_x");     
    const float off_y = getSignal("off_y");
    const float off_z = getSignal("off_z");

        //VLOG(1) << name << " " << is_dirty << " " << im_in->name << " " << im_in->is_dirty;

    const float wd = Config::inst()->im_width;
    const float ht = Config::inst()->im_height;

    cv::Mat in_p = (cv::Mat_<float>(3,4) << 
        0, wd, wd, 0, 
        0, 0,  ht, ht,
        0, 0, 0, 0);

    cv::Mat in_roi = in_p.t()(cv::Rect(0, 0, 2, 4)); //).clone();
    in_roi = in_roi.clone(); 
    cv::Mat out_roi = in_roi.clone();
 
    if (getSignal("manual_xy") < 0.5) {
      //////////////////////////////////////////////////
      /// This implements a standard rotozoom
      // move the image prior to rotation
      cv::Mat offset = (cv::Mat_<float>(3,4) << 
          off_x, off_x, off_x, off_x, 
          off_y, off_y, off_y, off_y,
          off_z, off_z, off_z, off_z);

      // shift the image after rotation
      cv::Mat center_m = (cv::Mat_<float>(3,4) << 
          center.x, center.x, center.x, center.x, 
          center.y, center.y, center.y, center.y,
          center.z, center.z, center.z, center.z);

      // Rotation matrices
      cv::Mat rotz = (cv::Mat_<float>(3, 3) <<
          cos(phi), -sin(phi), 0, 
          sin(phi),  cos(phi), 0,
          0, 0, 1);

      cv::Mat roty = (cv::Mat_<float>(3, 3) <<
          cos(theta),  0, sin(theta),  
          0, 1, 0,
          -sin(theta), 0, cos(theta) );

      cv::Mat rotx = (cv::Mat_<float>(3, 3) <<
          1,  0,        0,
          0,  cos(psi), sin(psi),  
          0, -sin(psi), cos(psi) );

      // TBD reformat the matrices so all the transposes aren't necessary

      // Transform into ideal coords
      //float fx = getSignal("fx");
      cv::Mat out_p = (in_p - offset).t() * rotx.t() * roty.t() * rotz.t() * scale; // + center_m.t();
    
      out_roi = out_p(cv::Rect(0, 0, 2, 4)).clone();
     
      // this moves the image away from the 0 plane
      const float z = getSignal("z");
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
          out_roi.at<float>(i,j) = z * out_p.at<float>(i,j) / (out_p.at<float>(i,2) + z) + center_m.at<float>(j,i);
        }
      }

      //
      setSignal("x0", out_roi.at<float>(0,0));
      setSignal("y0", out_roi.at<float>(0,1));
      setSignal("x1", out_roi.at<float>(1,0));
      setSignal("y1", out_roi.at<float>(1,1));
      setSignal("x2", out_roi.at<float>(2,0));
      setSignal("y2", out_roi.at<float>(2,1));
      setSignal("x3", out_roi.at<float>(3,0));
      setSignal("y3", out_roi.at<float>(3,1));
      //
      setSignal("z0", out_p.at<float>(0,2));
      setSignal("z1", out_p.at<float>(1,2));
      setSignal("z2", out_p.at<float>(2,2));
      setSignal("z3", out_p.at<float>(3,2));
    } else {   
      out_roi.at<float>(0,0) = getSignal("x0");
      out_roi.at<float>(1,0) = getSignal("x1");
      out_roi.at<float>(2,0) = getSignal("x2");
      out_roi.at<float>(3,0) = getSignal("x3");
      out_roi.at<float>(0,1) = getSignal("y0");
      out_roi.at<float>(1,1) = getSignal("y1"); 
      out_roi.at<float>(2,1) = getSignal("y2");
      out_roi.at<float>(3,1) = getSignal("y3"); 
    }

    //  cv::Mat out_p_2d = cv::Mat_<float>(4, 2);
    //  out_p_2d.at<float>(i,j) = ;

    cv::Mat out;
    // TBD make inter_nearest changeable
    cv::Mat transform = cv::getPerspectiveTransform(in_roi, out_roi);
    cv::warpPerspective(in, out, transform, 
        in.size(), getModeType(), getBorderType());
    setImage("out", out);

    #if 0
    for (int i = 0; i < transform.rows; i++) {
    for (int j = 0; j < transform.cols; j++) {
      std::string name = "m" + boost::lexical_cast<string>(i) + boost::lexical_cast<string>(j);
      setSignal(name, out_p.at<float>(i,j));
    }}
    #endif

    return true;
  }

  //
  Undistort::Undistort(const std::string name) : ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in",tmp);
    setSignal("fx", Config::inst()->im_width/2 );
    setSignal("fy", Config::inst()->im_width/2 );
    setSignal("cx", Config::inst()->im_width/2 );
    setSignal("cy", Config::inst()->im_height/2 );
    setSignal("d0", 0 );
    setSignal("d1", 0 );
    setSignal("d2", 0 );
    setSignal("d3", 0 );
    
    setSignal("d4", 0 );
    setSignal("d5", 0 );
    setSignal("d6", 0 );
    setSignal("d7", 0 );
  }

  bool Undistort::update()
  {
    if (!Node::update()) return false;
    
    if (!isDirty(this,40)) return true;

    cv::Mat in = getImage("in");
    if (in.empty()) return false;

    cv::Mat intrinsics = (cv::Mat_<float>(3,3) <<
        getSignal("fx"), 0, getSignal("cx"),
        0, getSignal("fy"), getSignal("cy"),
        0, 0, 1
        );
    // has to be size 4, 5, or 8
    // TBD with 1/100 scale factor
    cv::Mat dist = (cv::Mat_<float>(8,1) <<
        getSignal("d0")/100.0,
        getSignal("d1")/100.0,
        getSignal("d2")/100.0,
        getSignal("d3")/100.0,
        getSignal("d4")/100.0,
        getSignal("d5")/100.0,
        getSignal("d6")/100.0,
        getSignal("d7")/1000.0
        );

    cv::Mat out;

    cv::undistort(in, out, intrinsics, dist);
    setImage("out",out);
    return true;
  }

  Remap::Remap(const std::string name) :
      ImageNode(name)
  {
    cv::Mat in;
    setImage("in", in);
    cv::Mat offx;
    setImage("offx", offx);
    setSignal("scalex", 1.0);
    cv::Mat offy;
    setImage("offy", offy);
    setSignal("scaley", 1.0);
    setSignal("mode", 0, false, ROLL, 0, 4);

    base_x = cv::Mat( Config::inst()->getImSize(),
      CV_32FC1);
    base_y = base_x.clone();
    base_xy = cv::Mat( Config::inst()->getImSize(),
      CV_32FC2);

    for (int i = 0; i < base_x.rows; i++) {
    for (int j = 0; j < base_x.cols; j++) {
      base_x.at<float>(i,j) = j;
      base_y.at<float>(i,j) = i;
    }
    }

    int ch1[] = {0, 0};
    mixChannels(&base_x, 1, &base_xy, 1, ch1, 1);
    int ch2[] = {0, 1};
    mixChannels(&base_y, 1, &base_xy, 1, ch2, 1);

  } 

  bool Remap::update() 
  {
    if (!Node::update()) return false;
    
    if (!isDirty(this,40)) return true;

    cv::Mat in = getImage("in");
    if (in.empty()) return false;

    cv::Mat offx = getImage("offx");
    cv::Mat offy = getImage("offy");

    cv::Mat offx_scaled;
    cv::Mat offy_scaled;

    //if (offy.empty() && offx.empty()) {
    if (offy.empty() || offx.empty()) {
      setImage("out", in);
      return false;
    } /* else if (offy.empty() && !offx.empty()) {
      offy_scaled = cv::Mat(base_x.size(), CV_16SC1, cv::Scalar(0));
      offx.convertTo(offx_scaled, CV_16SC1, getSignal("scalex"));
    } else if (!offy.empty() && offx.empty()) {
      offx_scaled = cv::Mat(base_x.size(), CV_16SC1, cv::Scalar(0));
      offy.convertTo(offy_scaled, CV_16SC1, getSignal("scaley"));
    } else {
      offx.convertTo(offx_scaled, CV_16SC1, getSignal("scalex"));
      offy.convertTo(offy_scaled, CV_16SC1, getSignal("scaley"));
    }*/

    cv::Mat out;
  #if 1
    cv::Mat offx8, offy8;
    cv::cvtColor(offx, offx8, CV_RGB2GRAY);
    cv::cvtColor(offy, offy8, CV_RGB2GRAY);
      
    offx8.convertTo(offx_scaled, CV_32FC1, getSignal("scalex"));
    offy8.convertTo(offy_scaled, CV_32FC1, getSignal("scaley"));
    
    cv::Mat dist_x = base_x + offx_scaled;
    cv::Mat dist_y = base_y + offy_scaled;
  
    cv::Mat dist_xy16, dist_int;
    cv::convertMaps(dist_x, dist_y, dist_xy16, dist_int, CV_16SC2, true);

    cv::remap(in, out, dist_xy16, cv::Mat(), getModeType(), cv::BORDER_REPLICATE);
   #else
    cv::remap(in, out, base_x, base_y, getModeType() );
    #endif
    setImage("out", out);

    return true;
  }

  ////////////////////////////////////////////////////////////
  Webcam::Webcam(const std::string name) : 
      ImageNode(name),
      error_count(0)
  {
    setSignal("mode", 0, false, ROLL, 0, 4);

    LOG(INFO) << "camera opening ...";
    capture = VideoCapture(0); //CV_CAP_OPENNI );
    LOG(INFO) << "done.";

    if ( !capture.isOpened() ) {
      LOG(ERROR) << "Can not open a capture object.";
      return;// -1;
    }

        //bool rv1 = capture.set( CV_CAP_PROP_FRAME_WIDTH, 800);
        //    //bool rv2 = capture.set( CV_CAP_PROP_FRAME_HEIGHT, 600);
        //        //LOG(INFO) << "set res " << rv1 << " " << rv2;
        //
        //
    

    //update();
    //update();
    is_thread_dirty = false;
    cam_thread = boost::thread(&Webcam::runThread, this);

    // wait for single frame so there is a sample with the correct size
    // TBD or time out
    while(!is_thread_dirty && (error_count < 20)) {}
  }
  
  Webcam::~Webcam()
  {
    LOG(INFO) << name << " stopping camera capture thread";
    do_capture = false;
    run_thread = false;
    cam_thread.join();

    LOG(INFO) << name << " releasing the camera";
    capture.release();
  }

  void Webcam::runThread()
  {
    do_capture = true;
    run_thread = true;

    //cv::namedWindow("webcam", CV_GUI_NORMAL);


    while(run_thread) {
      if (do_capture && error_count < 20) {
        /// TBD need to make this in separate thread
        if( !capture.grab() )
        {
          LOG(ERROR) << name << " Can not grab images." << endl;
          error_count++;
          continue;
          //return false;
        } 

        cv::Mat new_out;
        capture.retrieve(new_out);

        if (new_out.empty()) {
          LOG(ERROR) << name << " new image empty";
          error_count++;
          continue;
          //return false;
        }

        error_count--;
        if (error_count < 0) error_count = 0;
        // I think opencv is reusing a mat within capture so have to clone it

        //if (&new_out.data == &out.data) {
        //
        //cv::Mat tmp; // new_out.clone();
       
        float scale = 1.0;
        if (MAT_FORMAT == CV_16S) scale = 255;
        if (MAT_FORMAT == CV_32F) scale = 1.0/255.0;
        if (MAT_FORMAT == CV_8U) scale = 1.0;
        cv::Mat tmp0 = cv::Mat(new_out.size(), CV_8UC4, cv::Scalar(0)); 
        // just calling reshape(4) doesn't do the channel reassignment like this does
        int ch[] = {0,0, 1,1, 2,2}; 
        mixChannels(&new_out, 1, &tmp0, 1, ch, 3 );
        cv::Mat tmp;
        tmp0.convertTo(tmp, MAT_FORMAT,scale); //, 1.0/(255.0));//*255.0*255.0*255.0));

        cv::Size sz = Config::inst()->getImSize();
        cv::Mat tmp1;
        cv::resize(tmp, tmp1, sz, 0, 0, getModeType() );
        

        //out_lock.lock();
        setImage("out", tmp1);
        is_thread_dirty = true;
        //out_lock.unlock();
        //} else {
        //  VLOG(3) << name << " dissimilar capture";
        //  out = new_out;
        //}

        // TBD out is the same address every time, why doesn't clone produce a new one?
        //VLOG(3) << 

      } else {
        usleep(1000);
      }
    }
  } // runThread

/*
  cv::Mat get()
  {

  }
  */

  bool Webcam::update()
  {
    // don't call ImageNode update because it will clobber the "out" image set in the thread
    Node::update();
    //ImageNode::update();

      if ( is_thread_dirty ) setDirty();
      is_thread_dirty = false;

    return true;
  }


///////////////////////////////////////////////////////////

  ImageDir::ImageDir(const std::string name) : Buffer(name) 
  {
    setSignal("mode", 0, false, ROLL, 0, 4);
  }

  bool ImageDir::loadImages()
  {
    LOG(INFO) << name << " loading " << dir;

    
    boost::filesystem::path image_path(dir);
    if (!is_directory(image_path)) {
      LOG(ERROR) << name << CLERR << " not a directory " << CLNRM << dir; 
      return false;
    }

    // TBD clear frames first?
    
    vector<string> files;
    boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
    for (boost::filesystem::directory_iterator itr( image_path );
        itr != end_itr;
        ++itr )
    {
      if ( is_directory( *itr ) ) continue;
      
      stringstream ss;
      ss << *itr;
      string next_im = ( ss.str() );
      // strip off "" at beginning/end
      next_im = next_im.substr(1, next_im.size()-2);
      files.push_back(next_im);
   }

   frames_orig.clear();

   sort(files.begin(), files.end());
  
   for (int i=0; i < files.size(); i++) {
      const string next_im = files[i];
      cv::Mat new_out = cv::imread( next_im );
   
      if (new_out.data == NULL) { //.empty()) {
        LOG(WARNING) << name << " not an image? " << next_im;
        continue;
      }
   
     
      cv::Mat tmp0 = cv::Mat(new_out.size(), CV_8UC4, cv::Scalar(0)); 
        // just calling reshape(4) doesn't do the channel reassignment like this does
        int ch[] = {0,0, 1,1, 2,2}; 
        mixChannels(&new_out, 1, &tmp0, 1, ch, 3 );

        
      VLOG(1) << name << " " << i << " loaded image " << next_im;

      frames_orig.push_back(tmp0);
    }
    
    /// TBD or has sized increased since beginning of function?
    if (frames.size() == 0) {
      LOG(ERROR) << name << CLERR << " no images loaded" << CLNRM;
      return false;
    }
    
    LOG(INFO) << name << " " << frames.size() << " image loaded";
    setSignal("ind", getSignal("ind"), false, ROLL, 0, frames.size()-1);
    //max_size = frames.size() + 1;
    setDirty();

    return true;
  } // loadImages

  bool ImageDir::resizeImages()
  {
    int mode = getModeType();
    //LOG(INFO) << "resize mode " << mode;

    boost::mutex::scoped_lock l(frames_mutex);
    frames.clear();
    
    for (int i = 0; i < frames_orig.size(); i++) {
      cv::Mat tmp0 = frames_orig[i]; 
      cv::Size sz = Config::inst()->getImSize();
      cv::Mat tmp1;
      cv::resize(tmp0, tmp1, sz, 0, 0, mode );

      const bool restrict_size = false;

      const bool rv = addCore(tmp1, restrict_size);
    }

    setDirty();
    return true;
  }

  bool ImageDir::load(cv::FileNodeIterator nd)
  {
    Buffer::load(nd);
    
    (*nd)["dir"] >> dir;

    loadImages();
    resizeImages();
  }

  bool ImageDir::save(cv::FileStorage& fs) 
  {
    Buffer::save(fs);

    fs << "dir" << dir;
  }

  bool ImageDir::update()
  {
    const bool rv = Buffer::update();
    if (!rv) return false;
    
    if (!isDirty(this, 27)) return true;

    if (getSignal("load") > 0.5) {
      loadImages();
      resizeImages();
    } else {
      Connector* con = NULL;
      string src_port;
      getInputPort(SIGNAL, "mode", con, src_port);
      
      if (con->isDirty(this, 48)) {
        resizeImages();
      }
    }
    // flush dirtiness
    isDirty(this, 27);

    return true;
  }

///////////////////////////////////////////////////////////
  Tap::Tap(const std::string name) : ImageNode(name)
  {
    vcol = cv::Scalar(100, 30, 250);

    getSignal("value");
    getBuffer("buffer",0);
    setInputPort(BUFFER,"buffer", NULL, "out");
    //getImage("Buffer");
  }

  void Tap::setup(Signal* new_signal, Buffer* new_buffer) 
  {
    // TBD need caller to provide these
    setInputPort(SIGNAL,"value", new_signal, "value");
    setInputPort(BUFFER,"buffer", new_buffer, "out");
  }

  bool Tap::update()
  {
    if (!Node::update()) return false;

    if (isDirty(this,4)) {
      float value = getSignal("value");     
      
      VLOG(5) << name << " update " << value;
      cv::Mat out; // = getImage("out");
      int actual_ind;
      out = getBuffer("buffer", value, actual_ind); //, tmp)) return false;
      setSignal("actual_ind", actual_ind);
      
      if (out.empty()) return false;

      setImage("out", out);
    }

    return true;
  }

  bool Tap::draw(cv::Point2f ui_offset) 
  {
    ImageNode::draw(ui_offset);
  }
  
  bool TapInd::update()
  {
    if (!Node::update()) return false;

    if (isDirty(this, 4)) {
      float value =  getSignal("value");     
      int ind = value;

      VLOG(2) << name << " update " << ind;
      cv::Mat out; //= getImage("out");
      int actual_ind;
      out = getBuffer("buffer", ind, actual_ind);
      setSignal("actual_ind", actual_ind);
      if (out.empty())  return false;

      setImage("out", out);
    }

    return true;
  }
  
  bool TapInd::draw(cv::Point2f ui_offset) 
  {
    Tap::draw(ui_offset);
    return true;
  }

  ///////////////////////////////////////////
  Add::Add(const std::string name) : 
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("add0", tmp);
    setSignal("add0", 1.0);
    vcol = cv::Scalar(200, 200, 50);
  }
  
  void Add::setup(vector<ImageNode*> np, vector<float> nf) 
  {
    if (np.size() != nf.size()) {
      LOG(ERROR) << CLWRN << "mismatched inputs and coefficients" << CLNRM;
      //return; 
    }
   
    // TBD instead of clearing all, only clear the keys that match "add"
    for (int i = 0; i < np.size(); i++) {
      const string port = "add" + boost::lexical_cast<string>(i);
      setInputPort(IMAGE, port, np[i], "out");
      setInputPort(SIGNAL, port, NULL, "value"); // this allows other signals to connect to replace nf
    }
  }

  bool Add::update()
  {
    if (!Node::update()) return false;

    //VLOG(1) << "name " << is_dirty << " " << p1->name << " " << p1->is_dirty << ", " << p2->name << " " << p2->is_dirty ;
    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      // TBD accomodate bad mats somewhere

      cv::Size sz;

      bool done_something = false;
      
      cv::Mat out;

      // TBD should these vectors just be stored with some incrementing string?
      // TBD loop through all input ImageNodes and input signals (or vector values, need to be able
      // to handle either)
      // TBD instead of having nf, loop through all svals and match on ones that start with add
      // and then run getSignal on those strings
      for (int i = 0; i < ports.size(); i++) {
        if (ports[i]->type != IMAGE) continue;

        const string port = ports[i]->name;
        
        if (port.substr(0,3) != "add") {
          VLOG(5) << name << " : " << port.substr(0,3) << " " << port;
          continue;
        }
        
        cv::Mat tmp_in;
        bool im_dirty;
        //const string port = "add" + boost::lexical_cast<string>(i);
        tmp_in = getImage(port);
        if (tmp_in.empty()) {
          //VLOG(5) << name << " : " << port << " image is empty"; 
          continue;
        }

        float val = getSignal(port);
        
        // with 8-bit unsigned it is necessary to have initial coefficients positive
        // zero minus anything will just be zero 
        if (!done_something) {
          out = tmp_in * val;
          sz = tmp_in.size();
          done_something = true;
        } else { 

          if (sz != tmp_in.size()) {
            LOG(ERROR) << name << " size mismatch " << sz.width << " " << sz.height 
                << " != " << tmp_in.size().width << " " << tmp_in.size().height ;
            continue;
          }
         
          if (val > 0)
            out += tmp_in * val;
          else 
            out -= tmp_in * -val;

        }
      } // nf loop
    
    //VLOG(1) << name << " " << "update";
    setImage("out", out);

    return true;
  }

  // TBD am duplicating this code a lot
  bool Add::handleKey(int key)
  {
    bool valid_key = ImageNode::handleKey(key);
    if (valid_key) return true;
   
    valid_key = true;
    if (key == '[') {
    
      // add an input addition port, TBD move to function
      int add_num = 0;
      for (int i = 0; i < ports.size(); i++) {
        if (ports[i]->type != IMAGE) continue;
        const string port = ports[i]->name;
        
        if (port.substr(0,3) != "add") {
          VLOG(1) << name << " : " << port.substr(0,3) << " " << port;
          continue;
        }
        add_num++;
      }

      // add a new addition port
      const string port = "add" + boost::lexical_cast<string>(add_num);
      setInputPort(IMAGE, port, NULL, "out");
      setInputPort(SIGNAL, port, NULL, "value"); // this allows other signals to connect to replace nf
       
      // TBD make a way to delete a port
    } else {
      valid_key = false;
    }

    // TBD 
    if (valid_key) setDirty();
    
    return valid_key;
  }

  //
  AddMasked::AddMasked(const std::string name) : 
      ImageNode(name)
  {
    cv::Mat tmp;
    setImage("add0", tmp);
    //setSignal("add0", 1.0);
    setImage("add1", tmp);
    //setSignal("add1", 1.0);
    setImage("mask", tmp);
    setSignal("offset", 0, false, SATURATE, 0, 255);
    vcol = cv::Scalar(200, 200, 50);
  }
 
  bool AddMasked::update()
  {
    if (!Node::update()) return false;

    //VLOG(1) << "name " << is_dirty << " " << p1->name << " " << p1->is_dirty << ", " << p2->name << " " << p2->is_dirty ;
    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }

    cv::Mat add0 = getImage("add0");
    if (add0.empty()) return true;
    cv::Mat add1 = getImage("add1");
    if (add1.empty()) return true;

    cv::Mat mask4 = getImage("mask");
    // derive the mask from the second input
    if (mask4.empty()) {
      // probably want to change add1 to black and white and then use it here,
      // otherwise color channels get masked individually. (a pixel that is 0 in red
      // but not in green will have different masks on those channels)
      mask4 = add1;
    } 
    
    int offset = getSignal("offset");
    cv::Mat mask = mask4 - cv::Scalar(offset,offset,offset,0); // cv::Mat(mask4.size(), CV_8UC1);
    // TBD use first channel as mask, TBD could combine all channels
    //int ch1[] = {0, 0};
    //mixChannels(&mask4, 1, &mask, 1, ch1, 1);
    
    //cv::Mat mask4_neg = 255 - mask4;
    
    // this is masking individually on all color channels, probably
    cv::Mat out = add0 & (mask == 0);
    out += add1 & (mask > 0);
    //cv::Add(add0,
    setImage("out", out);
    return true;
  }


  ////////////////////////////////////////
  Multiply::Multiply(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("mul0", tmp);
    setImage("mul1", tmp);
    setSignal("mul0", 1.0/16.0);
    setSignal("mul1", 1.0/16.0);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool Multiply::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      
    // TBD accomodate bad mats somewhere
    cv::Size sz;

    bool done_something = false;
      
    cv::Mat out;

    for (int i = 0 ; i < ports.size(); i++) {
      if (ports[i]->type != IMAGE) continue;
      const string port = ports[i]->name;

      if (port.substr(0,3) != "mul") {
        VLOG(1) << name << " : " << port.substr(0,3) << " " << port;
        continue;
      }

      cv::Mat tmp_in;
      bool im_dirty;
      tmp_in = getImage(port);
      if (tmp_in.empty()) {
        continue;
      }

      float val = getSignal(port);
      if (val < 0) {
        val = 0;
        setSignal(port, val);
      }
      
      if (!done_something) {
        out = tmp_in * val;
        sz = tmp_in.size();
        done_something = true;
      } else { 

        if (sz != tmp_in.size()) {
          LOG(ERROR) << name << " size mismatch " << sz.width << " " << sz.height 
            << " != " << tmp_in.size().width << " " << tmp_in.size().height ;
          continue;
        }

        out = out.mul(tmp_in * val);
      }
    } // port loop

    setImage("out", out);

    return true;
  }

  ////////////////////////////////////////
  AbsDiff::AbsDiff(const std::string name)  : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("diff0", tmp);
    setImage("diff1", tmp);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool AbsDiff::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      
    cv::Mat out;
    
    cv::Mat diff0 = getImage("diff0");
    cv::Mat diff1 = getImage("diff1");

    if (diff0.empty() && !diff1.empty()) {
      out = diff1;
    } else if (!diff0.empty() && !diff1.empty()) {
      out = diff0;
    } else if (diff0.size() != diff1.size()) {
      LOG(ERROR) << name << " size mismatch";
      return false;
    }

    cv::absdiff(diff0, diff1, out);

    setImage("out", out);

    return true;
  }

  ////////////////////////////////////////
  Greater::Greater(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in0", tmp);
    setImage("in1", tmp);
    vcol = cv::Scalar(200, 200, 50);
  }

  bool Greater::update()
  {
    if (!Node::update()) return false;

    if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
    }
      
    cv::Mat out;
    
    cv::Mat diff0 = getImage("in0");
    cv::Mat diff1 = getImage("in1");
    {
    boost::mutex::scoped_lock l(port_mutex);

    if (diff0.empty() && diff1.empty()) {
      return false;
    } else if (diff0.empty() && !diff1.empty()) {
      out = diff1;
    } else if (!diff0.empty() && diff1.empty()) {
      out = diff0;
    } else if (diff0.size() != diff1.size()) {
      LOG(ERROR) << name << " size mismatch";
      return false;
    } else if (diff0.type() != diff1.type()) {
      return false;
    } else if (diff0.size() == cv::Size(0,0)) {
      return false;
    } else {
      out = diff0 > diff1;
    }
    }
    /*
      TBD use actual function and have an int  input select among these options
CMP_EQ src1 equal to src2.
CMP_GT src1 greater than src2.
CMP_GE src1 greater than or equal to src2.
CMP_LT src1 less than src2.
CMP_LE src1 less than or equal to src2.
CMP_NE
      Or simply have a key that switches in and ref 
    */

    setImage("out", out);

    return true;
  }


  ////////////////////////////////////////
  Resize::Resize(const std::string name) : ImageNode(name) 
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("fx", 0.2);
    setSignal("fy", 0.2);
    setSignal("mode", 0, false, ROLL, 0, 4);
  }

  bool Resize::update()
  {
  if (!Node::update()) return false;
 
  if (!isDirty(this, 5)) { 
      VLOG(2) << name << " not dirty ";
      return true; 
  }
  cv::Mat in = getImage("in");

  if (in.empty()) {
    VLOG(2) << name << " in is empty";
    return false;
  }
  
  // if fx and fy aren't hooked up then they will remain unaltered

  cv::Size sz = in.size();
  
  float fx = abs(getSignal("fx"));
  float fy = abs(getSignal("fy"));
  
  cv::Size dsize = cv::Size(fx*sz.width, fy*sz.height);
  //TBD
  if (fx > 1.0) dsize.width = sz.width/fx;
  if (fy > 1.0) dsize.height = sz.height/fy;

  if (dsize.width < 1) dsize.width = 1;
  if (dsize.height < 1) dsize.height = 1;
  // TBD
  if (dsize.height > sz.height) dsize.height= sz.height;
  if (dsize.width > sz.width) dsize.width = sz.width;

  // scale image down
  cv::Mat tmp;
  cv::resize(in, tmp, dsize, 0, 0, cv::INTER_NEAREST);
  setSignal("sz_x", dsize.width);
  setSignal("sz_y", dsize.height);
  // then scale back to input size
  cv::Mat out;

  int mode_type = getModeType();
  
  // scale it back up to standard size
  cv::resize(tmp, out, sz, 0, 0, mode_type);
  setImage("out", out);
  VLOG(1) << fx << " " << fy << " " 
      << tmp.size().width << " " << tmp.size().height
      << " " << out.size().width << " " << out.size().height;
  return true;
  }

  ////////////////////////////////////////
  Flip::Flip(const std::string name) :
    ImageNode(name)
  {
    cv::Mat tmp;
    setImage("in", tmp);
    setSignal("flip_code", 0);
  }

  bool Flip::update()
  {
  if (!Node::update()) return false;
 
  // TBD make sure keyboard changed parameters make this dirty 
  if (!isDirty(this, 5)) { 
      VLOG(1) << name << " not dirty ";
      return true; 
  }
   
  cv::Mat in = getImage("in");

  if (in.empty()) {
    VLOG(2) << name << " in is empty";
    return false;
  }
  
  // if fx and fy aren't hooked up then they will remain unaltered

  cv::Size sz = in.size();
  
  int flip_code = getSignal("flip_code");
  while (flip_code > 2) flip_code -= 3;
  while (flip_code <-1) flip_code += 3;
  setSignal("flip_code", flip_code);

  cv::Mat out = cv::Mat(sz, in.type());

  if (flip_code == 2) out = in;
  else {
    flip(in, out, flip_code);
  }

  setImage("out", out);
  
  return true;
  }


  

} //bm

