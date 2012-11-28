/* cluster.c
 * Copyright (C) 2008 binarymillenium
 * This file is a Frei0r plugin.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __CLUSTER_H__
#define __CLUSTER_H__

#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <stdio.h>

#include "nodes.h"

namespace bm {

struct cluster_center
{
  int x;
  int y;

  int max_x;
  int min_x;
  int max_y;
  int min_y;

  cv::Vec4b rgb;

  /// aggregate color and positions
  float aggr_r;
  float aggr_g;
  float aggr_b;
  float aggr_x;
  float aggr_y;

  /// number of pixels in the cluster
  int numpix;
};

class Cluster : public ImageNode
{
  std::vector<cluster_center> clusters;
  
  public:
  Cluster();

  float find_dist(
		int r1, int g1, int b1, int x1, int y1,
		int r2, int g2, int b2, int x2, int y2,
		const float max_space_dist, const float dist_weight,
    const bool manhat= false); //, float color_weight)
  
  virtual bool update();
};

class PyrMean : public ImageNode
{
  public:
  PyrMean();

  virtual bool update();
};

}; // bm
#endif // __CLUSTER_H__
