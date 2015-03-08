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
    std::cout << this << " setting output to " << val << std::endl;
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
  // keep current output_
  if (coefficients_.size() == 0) return true;

  output_ = 0;
  //std::cout << this << " ";
  for (size_t i = 0; i < coefficients_.size() && i < inputs_.size(); ++i)
  {
    output_ += coefficients_[i] * inputs_[i]->getOutput();
    //std::cout << coefficients_[i] << " * " << inputs_[i]->getOutput() << " + ";
  }
  //std::cout << getLayer() << " output : " << output_ << std::endl;
  
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
  
  cv::Mat vis_;
  cv::RNG rng_;
  // will insert in order so updating can just go start to end
  std::vector<std::vector<nngen::Node*> > layers_;

};

Net::Net(std::vector<int> layer_sizes)
{
  std::vector<Node*> last_layer;

  vis_ = cv::Mat(cv::Size(600, 600), CV_8UC1, cv::Scalar::all(0));
  for (size_t j = 0; j < layer_sizes.size(); ++j)
  {
    std::vector<Node*> cur_layer;

    for (size_t i = 0; i < layer_sizes[j]; ++i)
    {
      std::vector<float> coefficients;
      // random for now
      for (size_t k = 0; k < last_layer.size(); ++k)
      {
        coefficients.push_back(rng_.gaussian(1.0));
      }

      Node* node = new Node(coefficients, last_layer);
      //std::cout << "layer " << j << " " << node->getLayer() << std::endl;
      cur_layer.push_back(node);
    }
  
    layers_.push_back(cur_layer);
    last_layer = cur_layer;
  }
}

bool Net::setInputs(std::vector<float> in_vals)
{
  if (layers_.size() == 0) return false;

  std::cout << "set input " << in_vals.size() << " " << layers_[0].size() << std::endl;
  for (size_t i = 0; (i < in_vals.size()) && (i < layers_[0].size()); ++i)
  {
    layers_[0][i]->setOutput(in_vals[i]);
  }
  return true;
}

bool Net::update()
{
  for (size_t i = 0; i < layers_.size(); ++i)
  {
    for (size_t j = 0; j < layers_[i].size(); ++j)
    {
      layers_[i][j]->update();
    }
  }
  return true;
}

std::string Net::print()
{
  if (layers_.size() == 0) return "no layers";
  std::stringstream ss;
  const size_t ind = layers_.size() - 1;
  for (size_t i = 0; i < layers_[ind].size(); ++i)
  {
    ss << layers_[ind][i]->getOutput() << " ";
  }
  return ss.str();
}

// visualize the network
bool Net::draw()
{
  
  cv::imshow("vis", vis_);
  return true;
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
  layer_sizes.push_back(2);
  layer_sizes.push_back(4);
  layer_sizes.push_back(8);
  //layer_sizes.push_back(12);
  //layer_sizes.push_back(16);
  nngen::Net* net = new nngen::Net(layer_sizes);

  cv::RNG rng;
  std::vector<float> in_vals;
  for (size_t i = 0; i < layer_sizes[0]; ++i) 
  {
    in_vals.push_back( rng.gaussian(1.0) + 1.0 );
    std::cout << in_vals[i] << " ";
  }
  std::cout << std::endl;
  net->setInputs(in_vals);
  
  net->update();
  std::cout << " output " << net->print() << std::endl;
  //net->draw();
  //cv::waitKey(0);
}
