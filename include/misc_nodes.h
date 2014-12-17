#ifndef __IMAGE_DIR_H__
#define __IMAGE_DIR_H__

#include <iostream>
#include <stdio.h>

#include <boost/thread.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//#include <random>
#include <deque>
#include <map>

#include "nodes.h"

namespace bm {

/////////////////////////////////
class BrowseDir : public ImageNode
{
  public:

  BrowseDir(const std::string name);
  virtual void init();
  
  virtual bool handleKey(int key);
  virtual bool update();
  
  private:
  boost::thread browse_thread;
  void runThread();
  
  boost::mutex dirs_mutex;
  std::vector<std::string> file_names;
  std::vector<std::string> sub_dirs;
  std::vector<int> num_sub_images;
  std::vector<int> num_sub_dirs;

};




} // bm
#endif // IMAGE_DIR
