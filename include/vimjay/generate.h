/** Copyright 2012 Lucas Walter */
#ifndef VIMJAY_GENERATE_H
#define VIMJAY_GENERATE_H

#include "vimjay/nodes.h"

#include <string>
/**
Nodes which generate images
*/

// #include <iostream>
// #include <stdio.h>
namespace bm
{

class Bezier : public ImageNode
{
public:
  explicit Bezier(const std::string name);
  virtual void init();
  virtual bool update();
};

class Circle : public ImageNode
{
public:
  explicit Circle(const std::string name);
  virtual void init();
  virtual bool update();
};

class Noise : public ImageNode
{
public:
  explicit Noise(const std::string name);
  virtual void init();
  virtual bool update();
};

class SimplexNoise : public ImageNode
{
public:
  explicit SimplexNoise(const std::string name);
  virtual void init();
  virtual bool update();
};
}  // namespace bm
#endif  // VIMJAY_GENERATE_H
