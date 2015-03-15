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

// weights are going to be reused so they need to be pointers
class Weight
{
public:
  Weight();
  float val_;
};

Weight::Weight()
{

}

class Node
{
public:
  void update();
  std::vector<std::vector< Node* > > inputs_;
  // TODO maybe this should just be a pointer to a cv::Mat
  std::vector<std::vector< Weight* > > weights_;
  float val_;

};

void Node::update()
{
  val_ = 0;

  // first layer
  for (size_t y = 0; y < inputs_.size(); ++y)
  {
    for (size_t x = 0; x < inputs_.size(); ++x)
    {
      val_ += inputs_[y][x]->val_ * weights_[y][x]->val_; 
    }
  }

  // TBD apply sigmoid or tanh
}

class Net
{
public:

  Net(cv::Mat& im);
  void update();

  std::vector<std::vector< Node* > > inputs_;
  std::vector<std::vector<std::vector< Node* > > > layer2_;

  std::vector< std::vector< std::vector < Weight* > > > bases_;
  
  // all in linear fasion
  std::vector<Node*> nodes_;
  std::vector<Weight*> weights_;
};

Net::Net(cv::Mat& im)
{
  inputs_.resize(im.rows);
  const float sigma = 0.5;


  // layer 1, just a copy of all the image pixels with no inputs
  for (size_t y = 0; y < im.rows; ++y)
  {
    inputs_[y].resize(im.rows);
    for (size_t x = 0; x < im.cols; ++x)
    {
      Node* node = new Node();
      node->val_ = im.at<uchar>(y,x);
      inputs_[y][x] = node;
      nodes_.push_back(node);
    }
  }

  cv::RNG rng;
  // create a basis image set of weights
  bases_.resize(1);
  for (size_t i = 0; i < bases_.size(); ++i)
  {
    bases_[i].resize(16);
    for (size_t y = 0; y < bases_[i].size(); ++y)
    {
      bases_[i][y].resize(16);
      for (size_t x = 0; x < bases_[i][y].size(); ++x)
      {
        Weight* weight = new Weight(); 
        weight->val_ = rng_.gaussian(sigma);
        bases_[i][y][x] = weight;
        weights_.push_back(weight);
      }
    }
  }

  // Layer 2
  // now create a reduced size node with 16x16 inputs
  const int div = 4;
  const size_t layer2_width = im.cols/div;
  const size_t layer2_height = im.rows/div;
  
  layer2_.resize(bases_.size());
  for (size_t i = 0; i < bases_.size(); ++i)
  {
    layer2_[i].resize(layer2_height);
    for (size_t y = 0; y < layer2_height; ++y)
    {
      layer2_[i][y].resize(layer2_width);
      for (size_t x = 0; x < layer2_width; ++x)
      {
        
      //
        Node* node = new Node();
        node->inputs_.resize(bases_[i].size());
        node->weights_.resize(bases_[i].size());

        for (int k = 0; k < bases_[i].size(); ++k)
        {
          node->inputs_.resize(bases_[i][k].size());
          node->weights_.resize(bases_[i][k].size());
          
          for (int l = 0; l < bases_[i].size(); ++l)
          {
            int y2 = y * div + k - bases_[i].size()/2; 
            int x2 = x * div + k - bases_[i][k].size()/2; 
            
            if ((y2 > 0) && (y2 < im.rows) &&
                (x2 > 0) && (x2 < im.cols))
            {
              node->inputs_[k][l] = node;
              node->weights_[k][l] = bases_[i][k][l];
              node->inputs_[k][l] = inputs_[y2][x2];
            }
          }
        }
       
        layer2_[i][y][x] = node;
        nodes_.push_back(node); 
      }
      //

    }
  }
}

void Net::update()
{
  for (size_t i = 0; i < nodes_.size(); ++i)
  {
    // all the nodes are in layer order so this works,
    // but later probably want a 2d array to make this more explicit
    nodes_->update();
  }
}

void Net::draw()
{
  cv::Mat layer2 = cv::Mat(cv::Size(layer2_[0].size(), layer2_.size()), CV_8UC1);
  
  for (size_t y = 0; y < layer2.cols; ++y)
  {
    for (size_t x = 0; x < layer2.cols; ++x)
    {
      layer2.at<uchar>(y, x) = 0;
    }
  }
  
  cv::imshow("net layer 2", layer2);
  
}

int main(int argn, char** argv)
{
  cv::Mat im = cv::imread("beach_128.png");
  cv::Mat yuv_im;

  cv::cvtColor(im, yuv_im, CV_RGB2YCrCb);
  std::vector<cv::Mat> yuvs;
  cv::split(yuv_im, yuvs);

  cv::imshow("input", yuvs[0]);
  Net* net = new Net(yuvs[0]);
  net->draw();
  cv::waitKey(0);

  return 0;
}

