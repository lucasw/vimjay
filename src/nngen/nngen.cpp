/*
  
  Copyright 2015 Lucas Walter

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
 

 Generate an image with a set of basis low resolution images and a neural network
 like set of layers than turn inputs (which could be mapped to gui sliders, or 
 keyboard keys, gamepad analog sticks) into images.

  g++ nngen.cpp -lopencv_highgui -lopencv_core -g && ./a.out 

 */

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

namespace nngen 
{

/* 
This will be an acyclic directed graph unlike the vimjay core stuff,
though introducing cyclic aspects would be interesting (but pollute the core
idea here?).

*/
class Node
{
public:
  Node(std::vector<float> coefficients, std::vector<Node*> inputs);

 
  bool update();
  void setOutput(const float val)
  {
    output_ = val;
  }
  float getOutput()
  {
    return output_;
  }
  int getLayer();
private:
  // these shouldn't change after setup?
  // make these a pair?
  std::vector<float> coefficients_;   
  std::vector<Node*> inputs_;  

  float output_;
};

Node::Node(std::vector<float> coefficients,
    std::vector<Node*> inputs) :
  coefficients_(coefficients),
  inputs_(inputs),
  output_(0.0)
{


}

int Node::getLayer()
{
  int count = 0;
  Node* node = this;
  while (node->inputs_.size() > 0) 
  {
    node = node->inputs_[0];
    count += 1;
  }
  return count;
}

bool Node::update()
{
  output_ = 0;
  for (size_t i = 0; i < coefficients_.size() && i < inputs_.size(); ++i)
  {
    output_ += coefficients_[i] * inputs_[i]->getOutput();
  }
  
  std::cout << getLayer() << " : " << output_ << std::endl;
  return true;
}

/////////////////////////////////////////////////////
class Net
{
public:
  Net(std::vector<int> layer_sizes);

  bool update();
  bool draw();
  
  bool setInputs(std::vector<float> in_vals);
  std::string print();

private:

  cv::RNG rng_;
  // will insert in order so updating can just go start to end
  std::vector<nngen::Node*> all_nodes_;
  std::vector<nngen::Node*> inputs_;
  // all the nodes inbetween are hidden
  std::vector<nngen::Node*> outputs_;

};

Net::Net(std::vector<int> layer_sizes)
{

  std::vector<Node*> last_layer;
  
  for (size_t j = 0; j < layer_sizes.size(); ++j)
  {
    std::vector<Node*> cur_layer;

    std::vector<float> coefficients;
    // random for now
    for (size_t k = 0; k < last_layer.size(); ++k)
    {
      coefficients.push_back(rng_.gaussian(1.0));
    }

    for (size_t i = 0; i < layer_sizes[j]; ++i)
    {
      Node* node = new Node(coefficients, last_layer);
      if (j == 0) 
        inputs_.push_back(node);
      all_nodes_.push_back(node);
      last_layer.push_back(node);
      if (j == layer_sizes.size() - 1) 
        outputs_.push_back(node);
    }
  
    last_layer = cur_layer;
  }
}

bool Net::setInputs(std::vector<float> in_vals)
{
  for (size_t i = 0; i < in_vals.size() && i < inputs_.size(); ++i)
  {
    inputs_[i]->setOutput(in_vals[i]);
  }
}

bool Net::update()
{
  for (size_t i = 0; i < all_nodes_.size(); ++i)
  {
    all_nodes_[i]->update();
  }
  return true;
}

std::string Net::print()
{
  std::stringstream ss;
  for (size_t i = 0; i < outputs_.size(); ++i)
  {
    ss << outputs_[i]->getOutput() << " ";
  }
  return ss.str();
}

///////////////////////////////////////
class ImageNet
{
  public:
  ImageNet(std::vector<int> layer_sizes);
  bool update();
  void draw();
  private:

  Net* net_;
  cv::Mat output_;
  std::vector<cv::Mat> bases;
};

ImageNet::ImageNet(std::vector<int> layer_sizes)
{
  output_ = cv::Mat(cv::Size(512,512), CV_8UC1, cv::Scalar::all(0));
  net_ = new Net(layer_sizes);
}

bool ImageNet::update()
{
  return net_->update();
}

void ImageNet::draw()
{
  cv::imshow("output", output_);
}

} // nngen


int main(int argn, char** argv)
{

  std::vector<int> layer_sizes;
  layer_sizes.push_back(4);
  layer_sizes.push_back(8);
  layer_sizes.push_back(12);
  layer_sizes.push_back(16);
  nngen::Net* net = new nngen::Net(layer_sizes);

  cv::RNG rng;
  std::vector<float> in_vals;
  for (size_t i = 0; i < layer_sizes[0]; ++i) 
  {
    in_vals.push_back( rng.gaussian(1.0) + 1.0 );
    std::cout << in_vals[i] << " ";
  }
  std::cout << std::endl;
  
  net->update();
  std::cout << " output " << net->print() << std::endl;
  //net->draw();
  //cv::waitKey(0);
}
