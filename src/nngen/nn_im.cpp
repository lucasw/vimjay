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

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "nn_im.hpp"


Weight::Weight() : 
    Base() 
{

}

Node::Node() :
    Base()
{
}
Node2d::Node2d() :
    Base()
{
}
Node3d::Node3d() :
    Base()
{
}

void Node::update()
{
  if (inputs_.size() == 0) return;

  val_ = 0;

  // first layer
  for (size_t y = 0; y < inputs_.size(); ++y)
  {
      if (inputs_[y] == NULL) continue;
      //std::cout << y << " " << x  << " " << inputs_[y].size() << " " 
      //    << weights_[y].size() <<   std::endl;
      //std::cout << inputs_[y][x] << std::endl;
      //std::cout << weights_[y][x] << std::endl;
      val_ += inputs_[y]->val_ * weights_[y]->val_; 
  }

  // TBD apply sigmoid or tanh
}

void Node2d::update()
{
  if (inputs_.size() == 0) return;
  if (inputs_[0].size() == 0) return;

  val_ = 0;

  // first layer
  for (size_t y = 0; y < inputs_.size(); ++y)
  {
    for (size_t x = 0; x < inputs_[y].size(); ++x)
    {
      if (inputs_[y][x] == NULL) continue;
      //std::cout << y << " " << x  << " " << inputs_[y].size() << " " 
      //    << weights_[y].size() <<   std::endl;
      //std::cout << inputs_[y][x] << std::endl;
      //std::cout << weights_[y][x] << std::endl;
      val_ += inputs_[y][x]->val_ * weights_[y][x]->val_; 
    }
  }

  // TBD apply sigmoid or tanh
}

void Node3d::update()
{
  if (inputs_.size() == 0) return;
  if (inputs_[0].size() == 0) return;
  if (inputs_[0][0].size() == 0) return;

  val_ = 0;

  // first layer
  for (size_t y = 0; y < inputs_.size(); ++y)
  {
    for (size_t x = 0; x < inputs_[y].size(); ++x)
    {
      for (size_t z = 0; z < inputs_[y][x].size(); ++z)
      {
        if (inputs_[y][x][z] == NULL) continue;
        //std::cout << y << " " << x  << " " << inputs_[y].size() << " " 
        //    << weights_[y].size() <<   std::endl;
        //std::cout << inputs_[y][x] << std::endl;
        //std::cout << weights_[y][x] << std::endl;
        val_ += inputs_[y][x][z]->val_ * weights_[y][x][z]->val_; 
      }
    }
  }

  // TBD apply sigmoid or tanh
}

///////////////////////////////////////////////////////////////////////////////
Net::Net(cv::Mat& im)
{
  const float sigma = 0.5;

  // layer 1, just a copy of all the image pixels with no inputs
  inputs_.resize(im.rows);
  for (size_t y = 0; y < im.rows; ++y)
  {
    inputs_[y].resize(im.rows);
    for (size_t x = 0; x < im.cols; ++x)
    {
      Node2d* node = new Node2d();
      node->val_ = float(im.at<uchar>(y,x))/256.0;
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
        weight->val_ = rng.gaussian(sigma);
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
        Node2d* node = new Node2d();
        node->inputs_.resize(bases_[i].size());
        node->weights_.resize(bases_[i].size());

        for (int k = 0; k < bases_[i].size(); ++k)
        {
          node->inputs_[k].resize(bases_[i][k].size());
          node->weights_[k].resize(bases_[i][k].size());
          
          for (int l = 0; l < bases_[i][k].size(); ++l)
          {
            int y2 = y * div + k - bases_[i].size()/2; 
            int x2 = x * div + l - bases_[i][k].size()/2; 
            
            if ((y2 > 0) && (y2 < im.rows) &&
                (x2 > 0) && (x2 < im.cols))
            {
              //std::cout << k << " " << l << 
              node->weights_[k][l] = bases_[i][k][l];
              node->inputs_[k][l] = inputs_[y2][x2];
            }
          }
        }
       
        layer2_[i][y][x] = node;
        nodes_.push_back(node); 
        //
      }

    }
  }

  // layer 3 decode the image
  // create all the nodes, but not their inputs yet
  layer3_.resize(im.rows);
  for (size_t y = 0; y < im.rows; ++y)
  {
    layer3_[y].resize(im.rows);
    for (size_t x = 0; x < im.cols; ++y)
    {
      Node3d* node = new Node3d();
      layer3_[y][x] = node;
      nodes_.push_back(node);
    }
  }

  // this is ridiculous
  for (size_t i = 0; i < bases_.size(); ++i)
  {
    for (size_t y = 0; y < layer2_height; ++y)
    {
      for (size_t x = 0; x < layer2_width; ++x)
      {
        for (int k = 0; k < bases_[i].size(); ++k)
        {
          for (int l = 0; l < bases_[i][k].size(); ++l)
          {
            const int y2 = y * div + k - bases_[i].size()/2; 
            const int x2 = x * div + l - bases_[i][k].size()/2; 
            
            if ((y2 > 0) && (y2 < im.rows) &&
                (x2 > 0) && (x2 < im.cols))
            {
              Node3d* node = dynamic_cast<Node3d*>(layer3_[i][y2][x2]);
              if (node == NULL) continue;
              // we know each node of output pixel has bases_.size()
              // amount of inputs into it times the number of layer 2
              // bases that overlap
              // at the current y2 x2 location
            }
          } 
        }
      }
    }
  }  
  // end ridiculous for loops
}

void Net::update()
{
  for (size_t i = 0; i < nodes_.size(); ++i)
  {
    // all the nodes are in layer order so this works,
    // but later probably want a 2d array to make this more explicit
    nodes_[i]->update();
  }
}

void layerToMat(std::vector< std::vector< Base* > >& layer, cv::Mat& vis)
{
  vis = cv::Mat(cv::Size(layer[0].size(), layer.size()), CV_8UC1);
  
  for (size_t y = 0; y < vis.rows; ++y)
  {
    for (size_t x = 0; x < vis.cols; ++x)
    {
      std::cout << y << " " << x << " " << layer[y][x]->val_ << std::endl;
      vis.at<uchar>(y, x) = layer[y][x]->val_ * 256 + 128;
    }
  } 
}

void Net::draw()
{
  {
    cv::Mat vis_pre;
    layerToMat(layer2_[0], vis_pre);
    cv::Mat vis;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * 8, vis_pre.rows * 8), 
        0, 0, cv::INTER_NEAREST);
    cv::imshow("net layer 2", vis);
  }

  {
    cv::Mat vis_pre;
    layerToMat(bases_[0], vis_pre);
    cv::Mat vis;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * 8, vis_pre.rows * 8), 
        0, 0, cv::INTER_NEAREST);
    cv::imshow("base 0", vis);
  }
}

///////////////////////////////////////////////////////////////////////////////
int main(int argn, char** argv)
{
  cv::Mat im = cv::imread("beach_128.png");
  cv::Mat yuv_im;

  cv::cvtColor(im, yuv_im, CV_RGB2YCrCb);
  std::vector<cv::Mat> yuvs;
  cv::split(yuv_im, yuvs);

  cv::imshow("input", yuvs[0]);
  Net* net = new Net(yuvs[0]);
  net->update();
  net->draw();
  cv::waitKey(0);

  return 0;
}

