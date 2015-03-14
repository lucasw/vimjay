/**
Lucas Walter
March 2015

GPL 3.0

Make a very small neural net and train it with google ceres.

Ideally this would take in an image of low-resolution (128x128) where each
pixel is an input into a node with 16x16 other inputs of neighbor pixels.
The coefficients to these inputs would be a 16x16 basis image.

The number of these nodes could be the same as the resolution of the image, or have a 4 pixel
spacing or greater.

Then on the second layer take the number of nodes down by a much greater factor
for instance down to 16x16.

Then the third layer generates a new image at 128x128.  To do this correctly the coefficients 
in the first layer ought to be duplicated in the third.  The incoming value
from the second layer scales the basis image coefficients of the first in other words.

  g++ nn_im.cpp -lopencv_imgproc -lopencv_highgui -lopencv_core && ./a.out 


*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class Node
{
public:
  void update();
  std::vector<std::vector< Node* > > inputs_;
  std::vector<std::vector< float > > weights_;
  float val_;

};

void Node::update()
{
  val_ = 0;
  for (size_t y = 0; y < inputs_.size(); ++y)
  {
    for (size_t x = 0; x < inputs_.size(); ++x)
    {
      val_ += inputs_[y][x]->val_ * weights_[y][x]; 
    }
  }

  // TBD apply sigmoid or tanh
}

class Net
{
public:

  Net(cv::Mat& im);
  
  std::vector<std::vector< Node* > > inputs_;
};

Net::Net(cv::Mat& im)
{
  
  for (size_t y = 0; y < im.rows; ++y)
  {
    for (size_t x = 0; x < im.cols; ++x)
    {
      Node* node = new Node();
      //net->val_ = im.at<
    }
  }

}

int main(int argn, char** argv)
{
  cv::Mat im = cv::imread("beach_128.png");
  cv::Mat yuv_im;

  cv::cvtColor(im, yuv_im, CV_RGB2YCrCb);
  std::vector<cv::Mat> yuvs;
  cv::split(yuv_im, yuvs);
  //Net* net = new Net(yuv_im);

  cv::imshow("input", yuvs[0]);

  cv::waitKey(0);

  return 0;
}

