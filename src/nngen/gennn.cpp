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

  g++ nngen.cpp -lopencv_imgproc -lopencv_highgui -lopencv_core && ./a.out 

 */

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

namespace nngen 
{

// http://stackoverflow.com/questions/23216247/how-to-convert-a-component-from-hsv-to-rgb-and-viceversa
cv::Scalar convertColor( cv::Scalar srcs, const int code, const int dstCn = 0 )
{
  cv::Vec3b src = cv::Vec3b(srcs[0], srcs[1], srcs[2]);
  cv::Mat srcMat(1, 1, CV_8UC3 );
  *srcMat.ptr< cv::Vec3b >( 0 ) = src;

  cv::Mat resMat;
  cv::cvtColor( srcMat, resMat, code, dstCn );

  cv::Vec3b res = *resMat.ptr< cv::Vec3b >( 0 );
  return cv::Scalar(res[0], res[1], res[2]);
}

/* 
This will be an acyclic directed graph unlike the vimjay core stuff,
though introducing cyclic aspects would be interesting (but pollute the core
idea here?).

*/
class Node
{
public:
  Node(std::vector<float> coefficients, std::vector<Node*> inputs);
  
  void setInputNodes(std::vector<Node*> inputs);
  bool update();
  void setOutput(const float val)
  {
    //std::cout << this << " setting output to " << val << std::endl;
    output_ = val;
  }
  float getOutput()
  {
    return output_;
  }
  int getLayer();
  
  cv::Point2f pos_;
  
  void draw(cv::Mat& vis);

  // TBD make private later
  std::vector<Node*> inputs_;  
 
private:
  // these shouldn't change after setup?
  // make these a pair?
  std::vector<float> coefficients_;   

  float output_;
};

Node::Node(std::vector<float> coefficients,
    std::vector<Node*> inputs) :
  coefficients_(coefficients),
  inputs_(inputs),
  output_(0.0)
{

}

void Node::setInputNodes(std::vector<Node*> inputs)
{
  inputs_ = inputs;
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

void Node::draw(cv::Mat& vis)
{
  for (size_t i = 0; i < inputs_.size(); ++i)
  {
    int val = coefficients_[i] * inputs_[i]->getOutput() * 64 + 128;
    if (val < 0) val = 0;
    if (val >= 255) val = 255;
    cv::Scalar col = convertColor( cv::Scalar(val, 228, 205), CV_HSV2BGR );
    //cv::line(vis, pos_, inputs_[i]->pos_, col, 1); 
  }
    
  int val = getOutput() * 64 + 128;
  if (val < 0) val = 0;
  if (val >= 255) val = 255;
  cv::Scalar col = convertColor( cv::Scalar(val, 228, 208), CV_HSV2BGR );
  //cv::Scalar col2 = 
  cv::circle(vis, pos_, 10, col, 1);

  std::stringstream txt;
  txt << getOutput();
  const int line_type = 8;
  const int font_face = cv::FONT_HERSHEY_SIMPLEX;
  const double font_scale = 0.3;
  //cv::putText(vis, txt.str(), pos_ + cv::Point2f(13, 4), font_face, font_scale,
  //    cv::Scalar(0,0,0), 2, line_type); 
  const cv::Scalar col2 = cv::Scalar(255, 230, 240);
  cv::putText(vis, txt.str(), pos_ + cv::Point2f(13, 4), font_face, font_scale,
      col2, 1, line_type); 
    
  
  const cv::Point2f offset = cv::Point2f(50, 8);
  cv::line(vis, pos_ + offset, pos_ + offset + cv::Point2f(getOutput() * 50, 0), col2, 2); 
}

bool Node::update()
{
  // keep current output_
  if (coefficients_.size() == 0) return true;

  float val = 0;
  //std::cout << this << " ";
  for (size_t i = 0; i < coefficients_.size() && i < inputs_.size(); ++i)
  {
    val += coefficients_[i] * inputs_[i]->getOutput();
    //std::cout << coefficients_[i] << " * " << inputs_[i]->getOutput() << " + ";
  }
  //std::cout << getLayer() << " output : " << output_ << std::endl;

  // make this nonlinear
  output_ = tanh(val); 

  return true;
}

/////////////////////////////////////////////////////
class Net
{
public:
  Net(std::vector<int> layer_sizes, const int seed, const float sigma, 
      const int num_inputs);

  bool update();
  bool draw();
  
  bool setInputs(std::vector<float> in_vals);
  bool getOutputs(std::vector<float>& out_vals);
  std::string print();
  
private:
  
  cv::Mat vis_;
  cv::RNG rng_;
  const int num_inputs_;
  // will insert in order so updating can just go start to end
  std::vector<std::vector<nngen::Node*> > layers_;

};

Net::Net(std::vector<int> layer_sizes, const int seed, const float sigma, 
  const int num_inputs) :
    rng_(seed),
    num_inputs_(num_inputs)
{
  std::vector<Node*> last_layer;

  vis_ = cv::Mat(cv::Size(700, 700), CV_8UC3, cv::Scalar::all(0));
  for (size_t j = 0; j < layer_sizes.size(); ++j)
  {
    std::vector<Node*> cur_layer;

    for (size_t i = 0; i < layer_sizes[j]; ++i)
    {
      std::vector<float> coefficients;
      // random for now
      for (size_t k = 0; k < last_layer.size(); ++k)
      {
        coefficients.push_back(rng_.gaussian(sigma));
      }

      Node* node = new Node(coefficients, last_layer);

      node->pos_ = cv::Point(30, 30) + cv::Point(120 * j, 23 * i);
      //std::cout << "layer " << j << " " << node->getLayer() << std::endl;
      cur_layer.push_back(node);
    }
  
    layers_.push_back(cur_layer);
    last_layer = cur_layer;
  }
  
  // recurrence (feedback output to input
  for (size_t i = num_inputs_; i < layers_[0].size(); ++i)
  {
    layers_[0][i]->setInputNodes(layers_[layers_.size() - 1]);
  }
}

bool Net::getOutputs(std::vector<float>& out_vals)
{
  if (layers_.size() == 0) return false;
  const int ind = layers_.size() - 1;
  out_vals.resize(layers_[ind].size());
  for (size_t i = 0; i < out_vals.size(); ++i)
  {
    out_vals[i] = layers_[ind][i]->getOutput();
  }
  return true;
}

bool Net::setInputs(std::vector<float> in_vals)
{
  if (layers_.size() == 0) return false;

  //std::cout << "set input " << in_vals.size() << " " << layers_[0].size() << std::endl;
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
  vis_ = cv::Scalar::all(0);
  
  const cv::Point offset = cv::Point(20,20);

  // draw nodes and connections
  for (size_t i = 0; i < layers_.size(); ++i)
  {
    for (size_t j = 0; j < layers_[i].size(); ++j)
    { 
      layers_[i][j]->draw(vis_);
    }
  }
  
  cv::imshow("vis", vis_);
  return true;
}

///////////////////////////////////////
class ImageNet
{
  public:
  ImageNet(std::vector<int> layer_sizes, const int num_inputs);
  bool update();
  void draw();
  private:

  int count_;
  int num_inputs_;

  void updateFromTrackbar();
  
  static const float sc_ = 64;
  Net* net_;
  Net* net_sc_;
  Net* net_x_;
  Net* net_y_;
  cv::Mat output_;
  std::vector<cv::Mat> bases_;
  //std::vector<cv::Mat> bases_big_;
};

ImageNet::ImageNet(std::vector<int> layer_sizes, const int num_inputs) :
  count_(0),
  num_inputs_(num_inputs)
{
  output_ = cv::Mat(cv::Size(480*3, 256*3), CV_8UC3, cv::Scalar::all(0));
  net_ = new Net(layer_sizes, 1, 0.14, num_inputs_);
  net_->update();
  net_sc_ = new Net(layer_sizes, 2, 0.8, num_inputs_);
  net_sc_->update();
  net_x_ = new Net(layer_sizes, 3, 0.20, num_inputs_);
  net_x_->update();
  net_y_ = new Net(layer_sizes, 4, 0.20, num_inputs_);
  net_y_->update();
  std::cout << " output " << net_->print() << std::endl;

  std::string prefix = "../../data/bases/";
  
  std::vector<std::string> base_files; 
  base_files.push_back("chess1.png");
  base_files.push_back("corner1.png");
  base_files.push_back("corner2.png");
  base_files.push_back("corner3.png");
  base_files.push_back("corner4.png");
  base_files.push_back("diag1.png");
  base_files.push_back("diag2.png");
  base_files.push_back("diag3.png");
  base_files.push_back("dood1.png");
  base_files.push_back("dood2.png");
  base_files.push_back("dood3.png");
  base_files.push_back("dot1.png");
  base_files.push_back("horiz1.png");
  base_files.push_back("noise1.png");
  base_files.push_back("noise2.png");
  base_files.push_back("noise3.png");
  base_files.push_back("slash1.png");
  base_files.push_back("slash2.png");
  base_files.push_back("slash3.png");
  base_files.push_back("slash4.png");
  base_files.push_back("vert1.png");

  for (size_t i = 0; i < base_files.size(); ++i)
  {
    cv::Mat base = cv::imread(prefix + base_files[i], CV_LOAD_IMAGE_COLOR);
    bases_.push_back(base);
    //cv::Mat base_resized;
    //const int mode = cv::INTER_CUBIC;
    //cv::resize(base, base_resized, cv::Size(output_.cols/3, output_.rows/3), 0, 0, mode);
    //bases_big_.push_back(base_resized);
  }
}

bool ImageNet::update()
{
  /*for (size_t i = 0; i < slider_vals_[0]; i++
  slider_vals_[0] = 256.0 + 256.0 * sin(float(count_)/45.1); 
  slider_vals_[1] = 256.0 + 256.0 * sin(float(count_)/193.1 + 0.1); 
  slider_vals_[2] = 256.0 + 256.0 * sin(float(count_)/321.523 + 0.2); 
  slider_vals_[3] = 256.0 + 256.0 * sin(float(count_)/551.12312 + 0.3); 
   */
  std::vector<float> in_vals;
  for (size_t i = 0; i < num_inputs_; ++i)
  {
    //std::cout << i << " " << slider_vals_[i] << std::endl;
    
    float freq = 1.0/(i * i * 201.1 + 115.1);
    float phase = i * 0.111;
    float val = sin(float(count_) * freq + phase);
    in_vals.push_back( val ); //(double) (slider_vals_[i] - 256) / 512.0 );
  }

  net_->setInputs(in_vals);
  net_sc_->setInputs(in_vals);
  net_x_->setInputs(in_vals);
  net_y_->setInputs(in_vals);

  const bool rv1 = net_->update();
  const bool rv2 = net_sc_->update();
  const bool rv3 = net_x_->update();
  const bool rv4 = net_y_->update();
  
  count_ += 1;
  //std::cout << count_ << std::endl;
  return rv1 && rv2 && rv3 && rv4;
}

void ImageNet::draw()
{
  //net_->draw();

  output_ = cv::Scalar::all(128);

  std::vector<float> out_vals;
  net_->getOutputs(out_vals);
 
  const float scale_fr = 0.2;
  const float xy_fr = 0.65;
  const float weight_fr = 0.95;

  std::vector<float> out_vals_sc;
  net_sc_->getOutputs(out_vals_sc);
  
  std::vector<float> out_vals_x;
  net_x_->getOutputs(out_vals_x);
  std::vector<float> out_vals_y;
  net_y_->getOutputs(out_vals_y);

  for (size_t i = 0; i < out_vals.size(); ++i)
  {
    const int base_ind = i % bases_.size();
    cv::Mat base = bases_[base_ind];
   
    const float fr = std::abs(out_vals_sc[i]);
    float sc2 = (0.05 + fr * scale_fr) * output_.rows/bases_[base_ind].rows;
    if (sc2 * bases_[base_ind].rows > output_.rows/3)
      sc2 = float(output_.rows/3) / float(bases_[base_ind].rows);
    
    cv::Mat base2;
    cv::resize(bases_[base_ind], base2, cv::Size(0,0), 
        sc2, sc2, 
        //cv::INTER_CUBIC ); 
        cv::INTER_LINEAR ); 
        ///cv::INTER_NEAREST ); 

    int off_x = out_vals_x[i] * output_.cols/2.0 * xy_fr;
    int off_y = out_vals_y[i] * output_.rows/2.0 * xy_fr;
    //std::cout << out_vals_sc[i] << " " << out_vals_x[i] << " " << out_vals_y[i] << std::endl;
    int cur_x = output_.cols/2 + off_x - base2.cols/2;
    int cur_y = output_.rows/2 + off_y - base2.rows/2;
    
    if (cur_x < 0) cur_x = 0;
    if (cur_y < 0) cur_y = 0;
    if (cur_x + base2.cols >= output_.cols)
      cur_x = output_.cols - base2.cols - 1;
    if (cur_y + base2.rows >= output_.rows)
      cur_y = output_.rows - base2.rows - 1;

    //std::cout << base_ind << " " << sc2 << " " << base2.size() << std::endl;
    cv::Rect roi = cv::Rect(cur_x, cur_y, base2.cols, base2.rows);

    cv::Mat dst_roi = output_(roi);
    cv::Mat scaled_base = base2 * std::abs(out_vals[i]) * weight_fr;
    //scaled_base.copyTo(dst_roi);
    if (out_vals[i] > 0)
      dst_roi += scaled_base;
    else
      dst_roi -= scaled_base;
    //cur_x += base.cols;
    
    //std::cout << roi << std::endl;
  }

  const int wd = output_.cols/3;
  const int ht = output_.rows/3;
  cv::Rect roi = cv::Rect(wd, ht, wd, ht);
  
  //cv::imshow("output", output_(roi));
  cv::imshow("output", output_); //(roi));

  if (false) {
    static int count = 100000;
    std::stringstream ss;
    ss << "imagenn_" << count << ".jpg";
    cv::imwrite(ss.str(), output_(roi));
    count += 1;
  }
}

} // nngen



int main(int argn, char** argv)
{
  std::vector<int> layer_sizes;
  //layer_sizes.push_back(4);
  layer_sizes.push_back(32);
  layer_sizes.push_back(16);
  layer_sizes.push_back(32);
  layer_sizes.push_back(32);
  layer_sizes.push_back(64);
  layer_sizes.push_back(128);
  layer_sizes.push_back(256);
  //layer_sizes.push_back(64);
  //layer_sizes.push_back(12);
  //layer_sizes.push_back(16);
  cv::namedWindow("vis");
  nngen::ImageNet* imnet = new nngen::ImageNet(layer_sizes, layer_sizes[0]/4);

  imnet->update();
  
  while (true)
  {
    imnet->update();
    imnet->draw();
    if (cv::waitKey(5) == 'q') break;
  }
}
