#ifndef NN_IM_HPP
#define NN_IM_HPP

// weights are going to be reused so they need to be pointers
class Base
{
public:
  Base();
  float val_;
  virtual void update() {}
};

Base::Base()
{

}

class Weight : public Base
{
public:
  Weight();
};

class Node: public Base
{
public:
  Node(int x, int y, int z);

  void drawGraph(cv::Mat& vis, const int sc);

  int x_;
  int y_; 
  int z_;
  
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
  Node2d(int x, int y, int z);
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
  Node3d(int x, int y, int z);
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
