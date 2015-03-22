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

Node::Node(int x, int y, int z) :
    x_(x),
    y_(y),
    z_(z),
    Base()
{
}

void Node::drawGraph(cv::Mat& vis, const int sc)
{
  cv::Point pt1 = cv::Point(x_ * sc, y_ * sc);
  
  for (size_t i = 0; i < outputs_.size(); ++i)
  {
    cv::Point pt2 = cv::Point(
        outputs_[i]->x_ * sc + rand()%sc/2, 
        outputs_[i]->y_ * sc + rand()%sc/2);
    const int weight = output_weights_[i]->val_ * 255;
    const int val = val_ * 255;
    cv::Scalar col = cv::Scalar(255, val, val);
    cv::line(vis, pt1, pt2, col, 1);
  }
}

Node2d::Node2d(int x, int y, int z) :
    Node(x, y, z)  
{
}
Node3d::Node3d(int x, int y, int z) :
    Node(x, y, z)  
{
}

void Node::update()
{
  if (inputs_.size() == 0) return;

  val_ = 0;

  // normalize
  float sum = 0;
  float count = 0;
  for (size_t y = 0; y < inputs_.size(); ++y)
  {
    if (inputs_[y] == NULL) continue;
    sum += inputs_[y]->val_; 
    count += 1.0;
  }
  
  const float mean = sum / count;
  sum = 0;
  for (size_t y = 0; y < inputs_.size(); ++y)
  {
    if (inputs_[y] == NULL) continue;
    const float diff = (inputs_[y]->val_ - mean); 
    sum += diff * diff;
  }
  float var = std::sqrt(sum);
  if (var == 0.0) var = 1.0;

  for (size_t y = 0; y < inputs_.size(); ++y)
  {
    if (inputs_[y] == NULL) continue;
      //std::cout << y << " " << x  << " " << inputs_[y].size() << " " 
      //    << weights_[y].size() <<   std::endl;
      //std::cout << inputs_[y][x] << std::endl;
      //std::cout << weights_[y][x] << std::endl;
    val_ += (inputs_[y]->val_ - mean) / var * weights_[y]->val_; 
  }

  // TBD apply sigmoid (map to 0.0-1.0) or tanh (-1.0 to 1.0)
}

void Node2d::setup()
{
  if (inputs2_.size() == 0) return;
  if (inputs2_[0].size() == 0) return;

  val_ = 0;

  for (size_t y = 0; y < inputs2_.size(); ++y)
  {
    for (size_t x = 0; x < inputs2_[y].size(); ++x)
    {
      inputs_.push_back(inputs2_[y][x]);
      weights_.push_back(weights2_[y][x]);
    }
  }

  // TBD apply sigmoid or tanh
}

void Node3d::setup()
{
  if (inputs3_.size() == 0) return;
  if (inputs3_[0].size() == 0) return;
  if (inputs3_[0][0].size() == 0) return;

  val_ = 0;

  for (size_t y = 0; y < inputs3_.size(); ++y)
  {
    for (size_t x = 0; x < inputs3_[y].size(); ++x)
    {
      for (size_t z = 0; z < inputs3_[y][x].size(); ++z)
      {
        inputs_.push_back(inputs3_[y][x][z]);
        weights_.push_back(weights3_[y][x][z]);
      }
    }
  }

  // TBD apply sigmoid or tanh
}

///////////////////////////////////////////////////////////////////////////////
Net::Net(cv::Mat& im) :
  im_(im)
{
  const float sigma = 0.5;
  
  // layer 1, just a copy of all the image pixels with no inputs
  inputs_.resize(im.rows);
  for (size_t y = 0; y < im.rows; ++y)
  {
    inputs_[y].resize(im.rows);
    for (size_t x = 0; x < im.cols; ++x)
    {
      Node2d* node = new Node2d(x, y, 0);
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
        if (i == 1) weight->val_ = float(x) / 16.0 - 0.5; //rng.gaussian(sigma);
        if (i == 0) weight->val_ = 1.0 - float(x) / 16.0 - 0.5; //rng.gaussian(sigma);
        if (i == 2) weight->val_ = float(y) / 16.0; //rng.gaussian(sigma);
        if (i == 3) weight->val_ = 1.0 - float(y) / 16.0; //rng.gaussian(sigma);
        if (i == 4) weight->val_ = 0.5; //rng.gaussian(sigma);
        bases_[i][y][x] = weight;
        weights_.push_back(weight);
      }
    }
  }

  // Layer 2
  // now create a reduced size node with 16x16 inputs
  const int div = 16;
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
        Node2d* node = new Node2d(x * div + div/2, y * div + div/2, i);
        node->inputs2_.resize(bases_[i].size());
        node->weights2_.resize(bases_[i].size());

        for (int k = 0; k < bases_[i].size(); ++k)
        {
          node->inputs2_[k].resize(bases_[i][k].size());
          node->weights2_[k].resize(bases_[i][k].size());
          
          const int x2 = x * div + k; // - bases_[i][k].size()/2; 
          for (int l = 0; l < bases_[i][k].size(); ++l)
          {
            const int y2 = y * div + l; //- bases_[i].size()/2; 
            //std::cout << y << " " << x << ", " 
            //    << k << " " << l << ", " 
            //    << y2 << " " << x2 << std::endl;
            
            if ((y2 >= 0) && (y2 < im.rows) &&
                (x2 >= 0) && (x2 < im.cols))
            {
              //std::cout << k << " " << l << 
              Base* input_weight = bases_[i][k][l];
              Node* input_node   = dynamic_cast<Node*>( inputs_[y2][x2] );
              
              if ((input_node == NULL))
              {
                std::cerr << "layer 2 " << y2 << " " << x2 << " " 
                    << input_weight << " " << input_node 
                    << std::endl;
                continue;
              }

              node->weights2_[k][l] = input_weight;  
              node->inputs2_[k][l] = input_node;
              
              input_node->output_weights_.push_back(input_weight);
              input_node->outputs_.push_back(node);
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
  // don't create any new weights, use same weights as on inputs
  layer3_.resize(im.rows);
  for (size_t y = 0; y < im.rows; ++y)
  {
    layer3_[y].resize(im.cols);
    for (size_t x = 0; x < im.cols; ++x)
    {
      Node* node = new Node(x, y, 0);
      layer3_[y][x] = node;
      nodes_.push_back(node);

      Node2d* node_enc = dynamic_cast<Node2d*>(inputs_[y][x]);
      //  now loop through outputs of the encoder node - every
      // output of the input node needs to be an input to 
      // this node
      for (size_t i = 0; i < node_enc->outputs_.size(); ++i)
      {
        //basis_ind = node->outputs_[i]->z_;
        // TBD replace this with Node::addOutput()
        Base* input_weight = node_enc->output_weights_[i];
        Node* input_node = dynamic_cast<Node*>( node_enc->outputs_[i] );
        
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

void layerToMat2D(std::vector< std::vector< Base* > >& layer, cv::Mat& vis) 
{
  cv::Mat vis_pre = cv::Mat(cv::Size(layer[0].size(), layer.size()), CV_32FC1);
  std::cout << "size " << vis_pre.size() << std::endl; 
  for (size_t y = 0; y < vis_pre.rows; ++y)
  {
    for (size_t x = 0; x < vis_pre.cols; ++x)
    {
      //std::cout << y << " " << x << " " << layer[y][x]->val_ << std::endl;
      vis_pre.at<float>(y, x) = layer[y][x]->val_; // * sc + offset;
    }
  }

  // now normalize
  cv::Scalar mean, std_dev;
  cv::meanStdDev(vis_pre, mean, std_dev);
  
  if (std_dev.val[0] != 0)
    vis_pre = (vis_pre - mean.val[0]) / std_dev.val[0] + 0.5;

  vis_pre.convertTo(vis, CV_8UC1, 255);
  
  cv::Scalar mean2, std_dev2;

  cv::meanStdDev(vis, mean2, std_dev2);

  std::cout << "mean, stddev ";
  std::cout << mean.val[0] << " " << std_dev.val[0] << ", ";
  std::cout << mean2.val[0] << " " << std_dev2.val[0] << std::endl;

}

bool layerToMat(std::vector< Base* >& layer, cv::Mat& vis)
{
  int xmin, xmax, ymin, ymax;
 
  // TODO make this a utility function?
  for (size_t i = 0; i < layer.size(); ++i)
  {
    Node* node = dynamic_cast<Node*>( layer[i] );
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
    //std::cerr << "bad wd ht " << xmax << " " << xmin << " " << ymax << " " << ymin << std::endl;
    //return false;
    wd = std::sqrt(layer.size());
    std::cout << "bad wd ht " << xmax << " " << xmin << " " << ymax << " " << ymin << std::endl;
    ht = wd;
    std::cout << "using " << wd << " " << ht << ", " << layer.size() << std::endl;
  }
  cv::Mat vis_pre = cv::Mat(cv::Size(wd, ht), CV_32FC1);
  
  for (size_t i = 0; i < layer.size(); ++i)
  {
    Node* node = dynamic_cast<Node*>( layer[i] );

    if (node == NULL) 
    {
      std::cerr << "bad node " << i << std::endl;
      continue;
    }
    //std::cout << y << " " << x << " " << layer[y][x]->val_ << std::endl;
    const int x = node->x_ - xmin;
    const int y = node->y_ - ymin;
    vis_pre.at<float>(y, x) = node->val_;
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


void Net::draw()
{
  std::cout << "output " << std::endl;
  {
    cv::Mat vis_pre;
    layerToMat2D(layer3_, vis_pre); // 128);
    cv::Mat vis;
    const int sc = 6;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc), 
        0, 0, cv::INTER_NEAREST);
    cv::imshow("output", vis);
  }

  std::cout << "layer2 " << std::endl;
  for (size_t i = 0; i < bases_.size(); ++i)
  {
    cv::Mat vis_pre;
    layerToMat2D(layer2_[i], vis_pre);
    cv::Mat vis;
    const int sc = 32;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc), 
        0, 0, cv::INTER_NEAREST);
    
    std::stringstream ss;
    ss << "layer2 " << i;
    cv::imshow(ss.str(), vis);
  }
  
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
 

  {
    std::cout << "input " << std::endl;
    cv::Mat vis_pre;
    layerToMat2D(inputs_, vis_pre);
    cv::Mat vis;
    const int sc = 4;
    cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc), 
        0, 0, cv::INTER_NEAREST);
    
    std::stringstream ss;
    ss << "input";
    cv::imshow(ss.str(), vis);
  }

  #if 0
  {
    const int sc = 16;
    cv::Mat vis = cv::Mat(cv::Size(im_.cols * sc, im_.rows * sc), CV_8UC3, 
        cv::Scalar::all(0)); 
    //cv::resize(im_, vis, cv::Size(), sc, sc, cv::INTER_NEAREST);
    
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
  //std::string image_file = "test_pattern.png"; 
  std::string image_file =  "beach_128.png";
  cv::Mat im = cv::imread(image_file);
  cv::Mat yuv_im;

  cv::cvtColor(im, yuv_im, CV_RGB2YCrCb);
  std::vector<cv::Mat> yuvs;
  cv::split(yuv_im, yuvs);

  Net* net = new Net(yuvs[0]);
  net->update();
  net->draw();

  int x = 0;
  int y = 0;
  
  while (true) 
  {
    {
      y = (y + net->layer3_.size()) % net->layer3_.size();
      x = (x + net->layer3_[y].size()) % net->layer3_[y].size();
      std::cout << "output input " << x << " " << y << std::endl;
      Node* node = dynamic_cast<Node*> (net->layer3_[y][x]);


      cv::Mat vis_pre;
      if (layerToMat( node->inputs_, vis_pre )) 
      {
      cv::Mat vis;
      const int sc = 16;
      cv::resize(vis_pre, vis, cv::Size(vis_pre.cols * sc, vis_pre.rows * sc), 
          0, 0, cv::INTER_NEAREST);

      std::stringstream ss;
      ss << "layer3 input";
      cv::imshow(ss.str(), vis);
      }
    }
   
    int key = cv::waitKey(0);
    if (key == 'q') break;

    if (key == 'j') y += 1;
    if (key == 'k') y -= 1;

    if (key == 'l') x += 1;
    if (key == 'h') x -= 1;
 

  }

  return 0;
}

