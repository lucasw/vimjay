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

#include <stdlib.h>
#include <assert.h>
#include <math.h>

#include <stdio.h>
#include <glog/logging.h>

#include "cluster.h"

namespace bm {

Cluster::Cluster()
{
  cv::Mat tmp;
  setImage("in", tmp);
	setSignal("dist_weight", 0.5);
  setSignal("margin", 0.3);
  setSignal("num", 5);
}

void initCluster(cluster_center& cc, int wd, int ht)
{
  cc.x = rand()%wd;
  cc.y = rand()%ht;

  int r = rand()%255;
  int g = rand()%255;
  int b = rand()%255;
  cc.rgb = cv::Vec4b(r,g,b,0);

  LOG(INFO) << cc.x << " " << cc.y << " " 
    << r << " " << g << " " << b << " "
    << cc.rgb[0] << " " << cc.rgb[1] << " " << cc.rgb[2];
  cc.numpix = 0;
  cc.aggr_x = 0;
  cc.aggr_y = 0;
  cc.aggr_r = 0;
  cc.aggr_g = 0;
  cc.aggr_b = 0;

  cc.max_x = 0;
  cc.max_y = 0;
  cc.min_x = wd;
  cc.min_y = ht;
}


float Cluster::find_dist(
		int r1, int g1, int b1, int x1, int y1,
		int r2, int g2, int b2, int x2, int y2,
		float max_space_dist, float dist_weight) //, float color_weight)
{
	/// make this a define?
	float max_color_dist = (255*255*3);

	float dr = r1-r2;
	float dg = g1-g2;
	float db = b1-b2;

	float color_dist = (dr*dr + dg*dg + db*db)/max_color_dist; 

	float dx = x1-x2;
	float dy = y1-y2;
	float space_dist = (dx*dx + dy*dy)/max_space_dist;

	/// add parameter weighting later
	return ((1.0-dist_weight)*color_dist*color_dist + dist_weight*space_dist*space_dist);
}

bool Cluster::update()
{
  Node::update();

  cv::Mat in = getImage("in");
  if (in.empty()) return false;
	
  float max_space_dist = sqrtf(in.cols + in.rows);

  cv::Mat out = in.clone();
	/*
	 if (has_initted) {
		has_initted = true;
	 }
	 */
  const float margin = getSignal("margin");
  const float upper = 1.0 + margin;
  const float lower = 1.0 - margin;

	const float dist_weight = getSignal("dist_weight");

  int num = getSignal("num");
  // 1 just tracks the average color of the image
  if (num < 1) num = 1; 
  
  // TBD arbitrary maximum
  if (num > 10) num = 10;
  const int old_size = clusters.size();
  clusters.resize(num);

  std::vector<cluster_center> nc;
  nc.resize(clusters.size());
  for (int k = 0; k < nc.size(); k++) {
    initCluster(nc[k], in.cols, in.rows);

    if (k >= old_size) {
      LOG(INFO) << old_size << " " << nc.size();
      initCluster(clusters[k], in.cols, in.rows); 
    }
  }

  for (int y=0; y < in.rows; ++y) {
  for (int x=0; x < in.cols; ++x) {
    
    cv::Vec4b src2 = in.at<cv::Vec4b> (y,x);

	  float dist = max_space_dist;
	  int dist_ind = 0;

    // search through all clusters for nearest one
	  for (int k = 0; k < clusters.size(); k++) {
		  struct cluster_center cc = clusters[k];	

		  if ((x < cc.max_x*upper+5) && (x > cc.min_x*lower-5) &&
				  (y < cc.max_y*upper+5) && (y > cc.min_y*lower-5)) {

			  const float kdist = find_dist(src2[0], src2[1], src2[2], x,y, 
					  cc.rgb[0], cc.rgb[1], cc.rgb[2], cc.x, cc.y,
					  max_space_dist, dist_weight); //, inst->color_weight);

        // store the closest match
			  if (kdist < dist) {
				  dist = kdist;
				  dist_ind = k;
			  }
		  }
	  } // clusters 

    // update min maxes
	  if (x > nc[dist_ind].max_x) nc[dist_ind].max_x = x;
	  if (x < nc[dist_ind].min_x) nc[dist_ind].min_x = x;
	  if (y > nc[dist_ind].max_y) nc[dist_ind].max_y = y;
	  if (y < nc[dist_ind].min_y) nc[dist_ind].min_y = y;

	  nc[dist_ind].aggr_x += x;
	  nc[dist_ind].aggr_y += y;
	  nc[dist_ind].aggr_r += src2[0];
	  nc[dist_ind].aggr_g += src2[1];
	  nc[dist_ind].aggr_b += src2[2];
	  nc[dist_ind].numpix += 1.0;

    // use the old cluster center color 
	  out.at<cv::Vec4b>(y,x) = clusters[dist_ind].rgb;

  }} // xy loop throug input image

  setImage("out", out);

  /// update cluster_centers
  for (int k = 0; k < nc.size(); k++) {
    if (nc[k].numpix <= 0) continue;

    nc[k].x = (int)  (nc[k].aggr_x/nc[k].numpix);
    nc[k].y = (int)  (nc[k].aggr_y/nc[k].numpix);
    nc[k].rgb = cv::Vec4b(
              (unsigned char) (nc[k].aggr_r/nc[k].numpix),
              (unsigned char) (nc[k].aggr_g/nc[k].numpix),
              (unsigned char) (nc[k].aggr_b/nc[k].numpix),
              0
              );
  }

  clusters = nc;
}

} //bm
