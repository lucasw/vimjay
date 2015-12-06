/**
Copyright 2015 Lucas Walter
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

#include <ceres/ceres.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
#include <vector>

#include "nn_im.hpp"

// NodeDiff class that has pointers to two nodes
// and reports the cost as the difference between them.
struct NodeDiffFunctor
{
  NodeDiffFunctor(
    void* net) :
    // see wasteful comment below
    // Node<T>* input,
    // Node<T>* output
    net_(net)
    // input_(input),
    // output_(output)
  {
  }

  // the input is actually every weight in the entire network
  template <typename T>
  bool operator()(const T* const* const* x, T* residual) const
  {
    // if x is pointing to the weights val (is that bad idea?)
    // then don't need to copy values here.
    // Net<T> net = dynamic_cast<Net<T>>

    // TBD need to copy the structure of net but have the type T
    // here.
    // Actually could reformulate the Net so that it never stores
    // a data type for val, just a network structure and then
    // there is a single function call that defines the type,
    // all the weights, and returns an output.
    Net<T>* net;

    // input array into data structure
    // iterate through all the weights and assign them values out of x-
    for (size_t i = 0; i < net->weights_.size(); ++i)
    {
      net->weights_[i]->val_ = x[i][0];
    }
    net->update();

    // this seems super wasteful, have to populate and update
    // everything just to get the diff of a single pixel
    // one al
    // residual = input_->val_ - output_->val_;

    // get the error of outputs minus inputs
    for (size_t i = 0; i < net->inputs_.size(); ++i)
    {
      residual[i] = net->layer3_[i]->val_ - net->inputs_[i]->val_;
    }

    return true;
  }

  // having this be a pointer seems like there is potential for
  // inter-thread conflict, TBD make it a copy instead
  void* net_;
  // const Node<T>* const input_;
  // const Node<T>* const output_;
};


template <typename T>
Weight<T>::Weight(std::string name) :
  Base<T>(name)
{
}

template <typename T>
Node<T>::Node(
  const std::string name,
  const int x, const int y, const int z,
  const bool use_var, const bool use_mean, const bool use_tanh) :
  x_(x),
  y_(y),
  z_(z),
  use_var_(use_var),
  use_mean_(use_mean),
  use_tanh_(use_tanh),
  Base<T>(name)
{
}

template <typename T>
void Node<T>::drawGraph(cv::Mat& vis, const int sc)
{
  cv::Point pt1 = cv::Point(x_ * sc, y_ * sc);

  for (size_t i = 0; i < outputs_.size(); ++i)
  {
    Node<T>* node = dynamic_cast<Node<T>*>(outputs_[i]);
    if (!node) continue;
    cv::Point pt2 = cv::Point(
                      node->x_ * sc + rand() % sc / 2,
                      node->y_ * sc + rand() % sc / 2);
    const int weight = static_cast<double>(output_weights_[i]->val_) * 255;
    const int val = this->val_ * 255;
    cv::Scalar col = cv::Scalar(255, val, val);
    cv::line(vis, pt1, pt2, col, 1);
  }
}

template <typename T>
Node2d<T>::Node2d(const std::string name,
                  const int x, const int y, const int z,
                  const bool use_var, const bool use_mean, const bool use_tanh) :
  Node<T>(name, x, y, z, use_var, use_mean, use_tanh)
{
}
template <typename T>
Node3d<T>::Node3d(const std::string name,
                  const int x, const int y, const int z,
                  const bool use_var, const bool use_mean, const bool use_tanh) :
  Node<T>(name, x, y, z, use_var, use_mean, use_tanh)
{
}

template <typename T>
void Node<T>::update()
{
  if (inputs_.size() == 0) return;

  this->val_ = 0;

  T var = 1.0;
  T mean = 0;
  // normalize
  if (use_mean_ && (inputs_.size() > 1))
  {
    T sum = 0;
    T count = 0;
    for (size_t y = 0; y < inputs_.size(); ++y)
    {
      if (inputs_[y] == NULL) continue;
      sum += inputs_[y]->val_;
      count += 1.0;
    }

    mean = sum / count;

    if (use_var_)
    {
      sum = 0;
      for (size_t y = 0; y < inputs_.size(); ++y)
      {
        if (inputs_[y] == NULL) continue;
        const T diff = (inputs_[y]->val_ - mean);
        sum += diff * diff;
      }
      // not averaging seems better.
      // var = (sum / count);
      if (var == 0.0) var = 1.0;

      // var *= 50.0;
    }
  }

  // std::cout << name_ << ", mean " << mean << ", var " << var << std::endl;
  for (size_t y = 0; y < inputs_.size(); ++y)
  {
    if (inputs_[y] == NULL) continue;
    // std::cout << y << " " << x  << " " << inputs_[y].size() << " "
    //    << weights_[y].size() <<   std::endl;
    // std::cout << inputs_[y][x] << std::endl;
    // std::cout << weights_[y][x] << std::endl;
    // std::cout << name_ << " input from " << inputs_[y]->name_ << " "
    //    << inputs_[y]->val_ << " * " << weights_[y]->val_ << std::endl;
    this->val_ += (inputs_[y]->val_ - mean) / var * weights_[y]->val_;
  }

  // std::cout << name_ << " val " <<  val_ << std::endl;
  // TBD apply sigmoid (map to 0.0-1.0) or tanh (-1.0 to 1.0)
  // std::cout << name_ << " val " <<  val_ << " " << tanh(val_) << std::endl;
  if (use_tanh_)
    this->val_ = tanh(this->val_);
}

template <typename T>
void Node2d<T>::setup()
{
  if (inputs2_.size() == 0) return;
  if (inputs2_[0].size() == 0) return;

  this->inputs_.resize(0);

  this->val_ = 0;

  for (size_t y = 0; y < inputs2_.size(); ++y)
  {
    for (size_t x = 0; x < inputs2_[y].size(); ++x)
    {
      this->inputs_.push_back(inputs2_[y][x]);
      this->weights_.push_back(weights2_[y][x]);
    }
  }

  // TBD apply sigmoid or tanh
}

template <typename T>
void Node3d<T>::setup()
{
  if (this->inputs3_.size() == 0) return;
  if (this->inputs3_[0].size() == 0) return;
  if (this->inputs3_[0][0].size() == 0) return;

  this->val_ = T(0);

  for (size_t y = 0; y < this->inputs3_.size(); ++y)
  {
    for (size_t x = 0; x < this->inputs3_[y].size(); ++x)
    {
      for (size_t z = 0; z < this->inputs3_[y][x].size(); ++z)
      {
        this->inputs_.push_back(inputs3_[y][x][z]);
        this->weights_.push_back(weights3_[y][x][z]);
      }
    }
  }

  // TBD apply sigmoid or tanh
}

///////////////////////////////////////////////////////////////////////////////
template <typename T>
Net<T>::Net(cv::Mat& im) :
  im_(im)
{
  const size_t base_sz = 16;
  const int div = 4;
  const int num_bases = 5;

  cv::RNG rng;
  const double sigma = 0.5;
  // create a basis image set of weights
  bases_.resize(num_bases);
  for (size_t i = 0; i < bases_.size(); ++i)
  {
    bases_[i].resize(base_sz);
    for (size_t y = 0; y < bases_[i].size(); ++y)
    {
      bases_[i][y].resize(base_sz);
      for (size_t x = 0; x < bases_[i][y].size(); ++x)
      {
        std::stringstream ss;
        ss << "base_" << i << "_" << y << "_" << x;
        Weight<T>* weight = new Weight<T>(ss.str());
        const double fx = static_cast<double>(x) / static_cast<double>(base_sz);
        const double fy = static_cast<double>(y) / static_cast<double>(base_sz);
        const double wn = sin(fx * M_PI) * sin(fy * M_PI);
        if (i == 0) weight->val_ = fx - 0.5;  // rng.gaussian(sigma);
        if (i == 1) weight->val_ = 1.0 - fx - 0.5;  // rng.gaussian(sigma);
        if (i == 2) weight->val_ = fy - 0.5;  // rng.gaussian(sigma);
        if (i == 3) weight->val_ = 1.0 - fy - 0.5;  // rng.gaussian(sigma);
        if (i == 4) weight->val_ = rng.gaussian(sigma);
        weight->val_ *= wn;
        bases_[i][y][x] = weight;
        weights_.push_back(weight);
      }
    }
  }

  // layer 1, just a copy of all the image pixels with no inputs
  std::cout << "layer1 size " << im.rows << " " << im.cols << std::endl;
  inputs_.resize(im.rows);
  for (size_t y = 0; y < im.rows; ++y)
  {
    inputs_[y].resize(im.cols);
    for (size_t x = 0; x < im.cols; ++x)
    {
      std::stringstream ss;
      ss << "layer1_" << y << "_" << x;
      Node2d<T>* node = new Node2d<T>(ss.str(), x, y, 0, true, true, true);
      node->val_ = static_cast<double>(im.at<uchar>(y, x)) / 128.0 - 1.0;
      inputs_[y][x] = node;
      nodes_.push_back(node);
    }
  }

  // Layer 2 - encoder
  // now create a reduced size node with 16x16 inputs
  const size_t layer2_width = im.cols / div;
  const size_t layer2_height = im.rows / div;
  std::cout << "layer2 size " << layer2_height << " " << layer2_width << std::endl;

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
        std::stringstream ss;
        ss << "layer2_" << i << "_" << y << "_" << x;
        Node2d<T>* node = new Node2d<T>(ss.str(), x * div + div / 2, y * div + div / 2, i, true, true, true);
        node->inputs2_.resize(bases_[i].size());
        node->weights2_.resize(bases_[i].size());

        for (int k = 0; k < bases_[i].size(); ++k)
        {
          node->inputs2_[k].resize(bases_[i][k].size());
          node->weights2_[k].resize(bases_[i][k].size());

          const int x2 = node->x_ + k - div / 2;
          for (int l = 0; l < bases_[i][k].size(); ++l)
          {
            const int y2 = node->y_ + l - div / 2;
            // std::cout << y << " " << x << ", "
            //    << k << " " << l << ", "
            //    << y2 << " " << x2 << std::endl;

            if ((y2 >= 0) && (y2 < im.rows) &&
                (x2 >= 0) && (x2 < im.cols))
            {
              Base<T>* input_weight = bases_[i][k][l];
              Node<T>* input_node   = dynamic_cast<Node<T>*>(inputs_[y2][x2]);

              if ((input_node == NULL))
              {
                std::cerr << "layer 2 " << y2 << " " << x2 << " "
                          << input_weight << " " << input_node
                          << std::endl;
                continue;
              }

              node->weights2_[k][l] = input_weight;
              // TBD need to have constant offset input nodes
              node->inputs2_[k][l] = input_node;

              input_node->output_weights_.push_back(input_weight);
              input_node->outputs_.push_back(node);

              // std::cout << "layer1 yx " << y2 << " " << x2 << std::endl;
            }
            else
            {
              // std::cout << "invalid layer1 yx " << y2 << " " << x2 << std::endl;
            }  // is input pixel valid or not
          }  // loop through bases_ pixels x
        }  // loop through bases_ pixels y

        layer2_[i][y][x] = node;
        nodes_.push_back(node);
        //
      }  // loop through layer2 x
    }  // loop through layer2 y
  }  // loop through bases


  // layer 3 decode the image
  // don't create any new weights, use same weights as on inputs
  layer3_.resize(inputs_.size());
  for (size_t y = 0; y < layer3_.size(); ++y)
  {
    layer3_[y].resize(inputs_[y].size());
    for (size_t x = 0; x < layer3_[y].size(); ++x)
    {
      std::stringstream ss;
      ss << "layer3_" << y << "_" << x;
      Node<T>* node = new Node<T>(ss.str(), x, y, 0, true, true, true);
      layer3_[y][x] = node;
      nodes_.push_back(node);

      Node2d<T>* node_enc = dynamic_cast<Node2d<T>*>(inputs_[y][x]);
      //  now loop through outputs of the encoder node - every
      // output of the input node needs to be an input to
      // this node
      for (size_t i = 0; i < node_enc->outputs_.size(); ++i)
      {
        // basis_ind = node->outputs_[i]->z_;
        // TBD replace this with Node::addOutput()
        Base<T>* input_weight = node_enc->output_weights_[i];
        Node<T>* input_node = dynamic_cast<Node<T>*>(node_enc->outputs_[i]);

        if ((input_node == NULL))
        {
          std::cerr << "layer 3 " << i << " "
                    << input_weight << " " << input_node
                    << std::endl;
          continue;
        }
        // TBD make a data structure to pair input and weight
        // TBD make a data structure to pair input and weight
        node->inputs_.push_back(input_node);
        node->weights_.push_back(input_weight);

        input_node->output_weights_.push_back(input_weight);
        input_node->outputs_.push_back(node);
      }
      // Debug look at outputs from layer1
      // std::cout << "layer1 " << y << " " << " " << x
      //    << ": in " << node_enc->inputs_.size()
      //    << " out " << node_enc->outputs_.size() << std::endl;
    }
  }

  //            Node3d* node = dynamic_cast<Node3d*>(layer3_[y2][x2]);
  //            if (node == NULL) continue;
  // we know each node of output pixel has bases_.size()
  // amount of inputs into it times the number of layer 2
  // bases that overlap
  // at the current y2 x2 location

  for (size_t i = 0; i < nodes_.size(); ++i)
  {
    nodes_[i]->setup();
  }

  // Debug look at outputs from layer2
  for (size_t i = 0; i < layer2_.size(); ++i)
  {
    for (size_t y = 0; y < layer2_[i].size(); ++y)
    {
      for (size_t x = 0; x < layer2_[i][y].size(); ++x)
      {
        Node<T>* input_node   = dynamic_cast<Node<T>*>(layer2_[i][y][x]);
        if ((input_node == NULL))
        {
          std::cerr << "layer 2 bad node " << i << " " << y << " " << x << " "
                    << std::endl;
          continue;
        }
        // std::cout << "layer2 " << i << " " << y << " " << " " << x
        //    << ": in " << input_node->inputs_.size()
        //    << ", out " << input_node->outputs_.size()
        //    << std::endl;
      }
    }
  }
}

template <typename T>
void Net<T>::update()
{
  for (size_t i = 0; i < nodes_.size(); ++i)
  {
    // all the nodes are in layer order so this works,
    // but later probably want a 2d array to make this more explicit
    nodes_[i]->update();
  }
}

template <typename T>
void layerToMat2D(std::vector< std::vector< Base<T>* > >& layer, cv::Mat& vis)
{
  cv::Mat vis_pre = cv::Mat(cv::Size(layer[0].size(), layer.size()), CV_32FC1);
  // std::cout << "size " << vis_pre.size() << std::endl;
  for (size_t y = 0; y < vis_pre.rows; ++y)
  {
    for (size_t x = 0; x < vis_pre.cols; ++x)
    {
      // std::cout << y << " " << x << " " << layer[y][x]->val_ << std::endl;
      vis_pre.at<double>(y, x) = layer[y][x]->val_;  // * sc + offset;
    }
  }

  // now normalize
  cv::Scalar mean, std_dev;
  cv::meanStdDev(vis_pre, mean, std_dev);
  double var = std::sqrt(std_dev.val[0]);
  if (var != 0)
    vis_pre = (vis_pre - mean.val[0]) / var + 0.5;

  vis_pre.convertTo(vis, CV_8UC1, 255);

  cv::Scalar mean2, std_dev2;

  cv::meanStdDev(vis, mean2, std_dev2);

  std::cout << "mean, stddev ";
  std::cout << mean.val[0] << " " << std_dev.val[0] << ", ";
  std::cout << mean2.val[0] << " " << std_dev2.val[0] << std::endl;
}

template <typename T>
bool layerToMat(std::vector< Base<T>* >& layer, cv::Mat& vis)
{
  int xmin = 0;
  int xmax = 0;
  int ymin = 0;
  int ymax = 0;

  // TODO(lucasw) make this a utility function?
  for (size_t i = 0; i < layer.size(); ++i)
  {
    Node<T>* node = dynamic_cast<Node<T>*>(layer[i]);
    if (node == NULL)
    {
      continue;
    }

    if (i == 0)
    {
      xmin = node->x_;
      xmax = node->x_;
      ymin = node->y_;
      ymax = node->y_;
    }

    if (node->x_ > xmax) xmax = node->x_;
    if (node->x_ < xmin) xmin = node->x_;
    if (node->y_ > ymax) ymax = node->y_;
    if (node->y_ < ymin) ymin = node->y_;
  }

  int wd = xmax - xmin;
  int ht = ymax - ymin;
  if ((wd == 0) || (ht == 0))
  {
    // std::cerr << "bad wd ht " << xmax << " " << xmin << " " << ymax << " " << ymin << std::endl;
    // return false;
    // TBD this isn't always a great choice, need to have caller determine this
    wd = std::sqrt(layer.size());
    std::cout << "can't use xy wd ht " << xmax << " " << xmin << " " << ymax << " " << ymin << std::endl;
    ht = wd;
    std::cout << "using " << wd << " " << ht << ", " << layer.size() << std::endl;
  }
  else
  {
    wd += 1;
    ht += 1;
    std::cout << "using wd ht from x_ y_ " << wd << " " << ht
              << ", " << ymax << " " << ymin << ", " << xmax << " " << xmin << std::endl;
  }

  if ((wd == 0) || (ht == 0))
  {
    std::cerr << "bad wd ht " << layer.size() << std::endl;
    return false;
  }
  cv::Mat vis_pre = cv::Mat(cv::Size(wd, ht), CV_32FC1);

  for (size_t i = 0; i < layer.size(); ++i)
  {
    Node<T>* node = dynamic_cast<Node<T>*>(layer[i]);

    size_t x, y;
    if (node == NULL)
    {
      y = i / wd;
      x = i % wd;
    }
    else
    {
      x = node->x_ - xmin;
      y = node->y_ - ymin;
    }
    if (y >= vis_pre.cols) continue;
    if (x >= vis_pre.rows) continue;
    // std::cout << "layer2Mat " << y << " " << x << " " << layer[i]->val_ << std::endl;
    vis_pre.at<double>(y, x) = layer[i]->val_;
  }

  // now normalize
  cv::Scalar mean, std_dev;
  cv::meanStdDev(vis_pre, mean, std_dev);

  if (std_dev.val[0] != 0)
    vis_pre = (vis_pre - mean.val[0]) / std_dev.val[0] + 0.5;

  vis_pre.convertTo(vis, CV_8UC1, 255);

  cv::Scalar mean2, std_dev2;

  cv::meanStdDev(vis, mean2, std_dev2);

  std::cout << mean.val[0] << " " << std_dev.val[0] << " ";
  std::cout << mean2.val[0] << " " << std_dev2.val[0] << std::endl;

  return true;
}


template <typename T>
void Net<T>::draw()
{
  std::cout << "output " << std::endl;
  {
    cv::Mat vis_pre;
    layerToMat2D(layer3_, vis_pre);  // 128);
    cv::Mat vis;
    const int sc = 512 / vis_pre.cols;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc),
               0, 0, cv::INTER_NEAREST);
    cv::imshow("output", vis);
  }

  for (size_t i = 0; i < layer2_.size(); ++i)
  {
    std::cout << "layer2 " << i << std::endl;
    cv::Mat vis_pre;
    layerToMat2D(layer2_[i], vis_pre);
    cv::Mat vis;
    const int sc = 256 / vis_pre.cols;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc),
               0, 0, cv::INTER_NEAREST);

    std::stringstream ss;
    ss << "layer2 " << i;
    cv::imshow(ss.str(), vis);
  }

#if 1
  std::cout << "bases " << std::endl;
  for (size_t i = 0; i < bases_.size(); ++i)
  {
    cv::Mat vis_pre;
    layerToMat2D(bases_[i], vis_pre);
    cv::Mat vis;
    const int sc = 8;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc),
               0, 0, cv::INTER_NEAREST);
    std::stringstream ss;
    ss << "base " << i;
    cv::imshow(ss.str(), vis);
  }
#endif

  {
    std::cout << "input " << std::endl;
    cv::Mat vis_pre;
    layerToMat2D(inputs_, vis_pre);
    cv::Mat vis;
    const int sc = 512 / vis_pre.cols;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc),
               0, 0, cv::INTER_NEAREST);

    std::stringstream ss;
    ss << "input";
    cv::imshow(ss.str(), vis);
  }

#if 0
  {
    const int sc = 512 / im_.cols;
    cv::Mat vis = cv::Mat(cv::Size(im_.cols * sc, im_.rows * sc), CV_8UC3,
                          cv::Scalar::all(0));
    // cv::resize(im_, vis, cv::Size(), sc, sc, cv::INTER_NEAREST);

    for (size_t i = 0; i < nodes_.size(); ++i)
    {
      nodes_[i]->drawGraph(vis, sc);
    }
    cv::imshow("graph", vis);
  }
#endif
}

///////////////////////////////////////////////////////////////////////////////
int main(int argn, char** argv)
{
  // std::string image_file = "test_pattern.png";
  // std::string image_file = "lena.png";  // "beach_128.png";
  std::string image_file = "beach_128.png";
  cv::Mat im = cv::imread(image_file, CV_LOAD_IMAGE_GRAYSCALE);

  if (im.empty())
  {
    std::cerr << "could not load " << image_file << std::endl;
    return -1;
  }
#if 0
  cv::Mat yuv_im;

  cv::cvtColor(im, yuv_im, CV_RGB2YCrCb);
  std::vector<cv::Mat> yuvs;
  cv::split(yuv_im, yuvs);

  Net* net = new Net(yuvs[0]);
#endif
  Net<double>* net = new Net<double>(im);

  std::vector<double*> parameter_blocks;
  for (size_t i = 0; i < net->weights_.size(); ++i)
  {
    parameter_blocks.push_back(&(net->weights_[i]->val_));
  }

#if 0
  // Newer version of ceres than 1.8.0 required here
  ceres::Problem problem;
  const int stride = 4;
  ceres::CostFunction* cost_function =
    new ceres::DynamicAutoDiffCostFunction<NodeDiffFunctor, stride>(new NodeDiffFunctor(net));
  // cost_function.AddParameterBlock(net->weights_.size());

  // cost_function->SetNumResiduals(net->inputs_.size());

  problem.AddResidualBlock(cost_function, NULL, parameter_blocks);
#endif

  net->update();
  net->draw();

  int i = 0;
  int x = 0;
  int y = 0;

  while (true)
  {
    {
      i = (i + net->layer2_.size()) % net->layer2_.size();
      y = (y + net->layer2_[i].size()) % net->layer2_[i].size();
      x = (x + net->layer2_[i][y].size()) % net->layer2_[i][y].size();
      std::cout << "output input " << i << " " << y << " " << x << std::endl;
      Node<double>* node = dynamic_cast<Node<double>*>(net->layer2_[i][y][x]);
      std::cout << "outputs " << node->outputs_.size() << std::endl;
      cv::Mat vis_pre;
      if (layerToMat(node->output_weights_, vis_pre))
      {
        std::cout << vis_pre.size() << std::endl;
        cv::Mat vis;
        const int sc = 256 / vis_pre.cols;
        cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc),
                   0, 0, cv::INTER_NEAREST);

        std::stringstream ss;
        ss << "layer2 output weights";
        cv::imshow(ss.str(), vis);
      }
      cv::Mat vis_pre2;
      if (layerToMat(node->outputs_, vis_pre2))
      {
        // TODO(lucasw) put this in layerToMat
        cv::Mat vis;
        const int sc = 256 / vis_pre2.cols;
        cv::resize(vis_pre2, vis, cv::Size(vis_pre2.cols * sc, vis_pre2.rows * sc),
                   0, 0, cv::INTER_NEAREST);

        std::stringstream ss;
        ss << "layer2 outputs";
        cv::imshow(ss.str(), vis);
      }
    }

    int key = cv::waitKey(0);
    if (key == 'q') break;

    if (key == 'j') y += 1;
    if (key == 'k') y -= 1;

    if (key == 'l') x += 1;
    if (key == 'h') x -= 1;

    if (key == 'd') i += 1;
    if (key == 'f') i -= 1;
  }

  return 0;
}

