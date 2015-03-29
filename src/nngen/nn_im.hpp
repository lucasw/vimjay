#ifndef NN_IM_HPP
#define NN_IM_HPP

// weights are going to be reused so they need to be pointers
class Base
{
public:
  Base(std::string name);
  std::string name_;
  float val_;
  virtual void update() {}
};

Base::Base(std::string name) :
  name_(name)
{

}

class Weight : public Base
{
public:
  Weight(std::string name);
};

class Node: public Base
{
public:
  Node(
      const std::string name, 
      const int x, const int y, const int z, 
      const bool usr_var, const bool use_mean, const bool use_tanh);

  void drawGraph(cv::Mat& vis, const int sc);
  
  // optionally scale with the variance-
  // TBD only makes sense if use_mean is true?
  const bool use_var_;
  // optionally subtract out the mean
  // setting this to false means that constant offsets
  // regions and not just edges can be represented
  const bool use_mean_;
  const bool use_tanh_;
  const int x_;
  const int y_; 
  const int z_;
  
  virtual void setup() {}
  virtual void update();
  std::vector< Base* > inputs_;
  std::vector< Base* > weights_;
  // TODO addOutput() and make these private
  std::vector< Base* > outputs_;
  std::vector< Base* > output_weights_;
};

class Node2d : public Node
{
public:
  ///Node2d();
  Node2d(
      const std::string name, 
      const int x, const int y, const int z, 
      const bool usr_var, const bool use_mean, const bool use_tanh);

  virtual void setup();
  // a std vector of Base is not the same as a vector of Node,
  // so have to use the base class
  std::vector<std::vector< Base* > > inputs2_;
  // TODO maybe this should just be a pointer to a cv::Mat
  std::vector<std::vector< Base* > > weights2_;
};

class Node3d : public Node
{
public:
  Node3d( 
      const std::string name, 
      const int x, const int y, const int z, 
      const bool usr_var, const bool use_mean, const bool use_tanh);
  
  virtual void setup();
  // a std vector of Base is not the same as a vector of Node,
  // so have to use the base class
  std::vector<std::vector<std::vector< Base* > > > inputs3_;
  // TODO maybe this should just be a pointer to a cv::Mat
  std::vector<std::vector<std::vector< Base* > > > weights3_;
};

void layerToMat(std::vector< std::vector< Base* > >& layer, cv::Mat& vis, const float sc);



class Net
{
public:

  Net(cv::Mat& im);
  void update();
  void draw();

  std::vector<std::vector< Base* > > inputs_;
  std::vector<std::vector<std::vector< Base* > > > layer2_;
  std::vector<std::vector< Base* > > layer3_;

  std::vector< std::vector< std::vector < Base* > > > bases_;
 
  cv::Mat im_;
  // all in linear fasion
  std::vector<Node*> nodes_;
  std::vector<Base*> weights_;
};

#endif // NN_IM_HPP
