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

#include <boost/timer.hpp>
#include <stdio.h>
#include <glog/logging.h>
#include <boost/lexical_cast.hpp>

#include "cluster.h"

namespace bm {

/**
TBD is most of same functionality perhaps with higher performance provided by
pyrMeanShiftFiltering?  It seems different in parameters but the result might end up looking very similar.

My manual method still may be useful for exposing some internal data for use elsewhere in the graph.
*/
Cluster::Cluster()
{
  cv::Mat tmp;
  setImage("in", tmp);
	setSignal("dist_weight", 0.5);
  setSignal("margin", 0.3);
  setSignal("num", 3);
  setSignal("wrap",0);
  setSignal("manhat",0);
  setSignal("time", 0);
}

void initCluster(cluster_center& cc, int wd, int ht, bool do_rand = false)
{
  if (do_rand) {
  cc.x = rand()%wd;
  cc.y = rand()%ht;

  int r = rand()%255;
  int g = rand()%255;
  int b = rand()%255;
  cc.rgb = cv::Vec4b(r,g,b,0);

  LOG(INFO) << cc.x << " " << cc.y << " " 
    << r << " " << g << " " << b << " "
    <<(int) cc.rgb.val[0] << " " << (int)cc.rgb.val[1] << " " << (int)cc.rgb.val[2];
    cc.max_x = wd;
    cc.max_y = ht;
    cc.min_x = 0;
    cc.min_y = 0;

  } else {
    cc.x = 0;
    cc.y = 0;
    cc.rgb = cv::Vec4b(0,0,0,0);

    cc.max_x = 0;
    cc.max_y = 0;
    cc.min_x = wd;
    cc.min_y = ht;
  }

  cc.numpix = 0;
  cc.aggr_x = 0;
  cc.aggr_y = 0;
  cc.aggr_r = 0;
  cc.aggr_g = 0;
  cc.aggr_b = 0;

}


float Cluster::find_dist(
		int r1, int g1, int b1, int x1, int y1,
		int r2, int g2, int b2, int x2, int y2,
		float max_space_dist, float dist_weight,
    const bool use_manhat) //, float color_weight)
{
	/// make this a define?
	float max_color_dist = (255*255*3);
  if (use_manhat) max_color_dist = 255*3;

	float dr = r1-r2;
	float dg = g1-g2;
	float db = b1-b2;

	float color_dist;
  if (!use_manhat) 
    color_dist = (dr*dr + dg*dg + db*db)/max_color_dist;
  else
    color_dist = (abs(dr) + abs(dg) + abs(db))/max_color_dist;

	float dx = x1-x2;
	float dy = y1-y2;
  
	float space_dist;
  if (!use_manhat)
    space_dist = (dx*dx + dy*dy)/max_space_dist;
  else
    space_dist = (abs(dx) + abs(dy))/max_space_dist;

	/// add parameter weighting later
	return ((1.0-dist_weight) * color_dist + dist_weight * space_dist);
}

bool Cluster::update()
{
  if (!Node::update()) return false;

  if (!isDirty(this, 40)) return true;
  
  boost::timer t1;

  cv::Mat in = getImage("in");
  if (in.empty()) return false;
	
  const bool use_manhat = getSignal("manhat") > 0.5;
  float max_space_dist = (in.cols*in.cols + in.rows*in.rows);
  if (use_manhat) max_space_dist = (in.cols + in.rows); 

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
      initCluster(clusters[k], in.cols, in.rows, true); 
      LOG(INFO) << k << " " << old_size << " " << nc.size() << ", " << clusters[k].x;
    }
  }

  const bool wrap = getSignal("wrap") > 0.5;
  const int wd = in.cols;
  const int ht = in.rows;

  for (int y = 0; y < in.rows; ++y) {
  for (int x = 0; x < in.cols; ++x) {
    
    cv::Vec4b src2 = in.at<cv::Vec4b> (y,x);

	  float dist = max_space_dist;
	  int dist_ind = 0;

    // search through all clusters for nearest one
	  for (int k = 0; k < clusters.size(); k++) {
		  struct cluster_center cc = clusters[k];	
  
      int x2 = x;
      int y2 = y;

      if (wrap) {
        float dx = cc.x - x2;
	      float dy = cc.y - y2;

        if (dx + wd < -dx) x2 -= wd;
        else if ( -(dx - wd) < dx) x2 += wd;
        
        if (dy + ht < - y2) y2 -= ht;
        else if ( -(dy - ht) < dy) y2 += ht;

      }
    
      // There might be an inevitable amount of oscillation in wrap mode where
      // points that might be within range in multiple directions
      // will flip-flop between them.
      // could try to smooth motion of colors and centers
      const float span_x = (cc.max_x - cc.min_x)*(upper)+10;
      const float span_y = (cc.max_y - cc.min_y)*upper;
      const int mid_x = (cc.max_x + cc.min_x)/2;
      const int mid_y = (cc.max_y + cc.min_y)/2;
		  if ((x2 < mid_x + span_x/2) && (x2 > mid_x - span_x/2) &&
				  (y2 < mid_y + span_y/2) && (y2 > mid_y - span_y/2)) {

			  const float kdist = find_dist(
            src2.val[0],   src2.val[1],   src2.val[2],   x2, y2, 
					  cc.rgb.val[0], cc.rgb.val[1], cc.rgb.val[2], cc.x, cc.y,
					  max_space_dist, dist_weight,
            use_manhat);
            //in.cols, in.rows);
            //,
            //wrap); //, inst->color_weight);

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
	  nc[dist_ind].aggr_r += src2.val[0];
	  nc[dist_ind].aggr_g += src2.val[1];
	  nc[dist_ind].aggr_b += src2.val[2];
	  nc[dist_ind].numpix += 1.0;

    // use the old cluster center color 
	  out.at<cv::Vec4b>(y,x) = clusters[dist_ind].rgb;
    // TBD optionally provide a scaled image that encodes distance from centers
	  //out.at<cv::Vec4b>(y,x) = cv::Vec4b(dist*1024, dist*512,
    //    dist*256, 0);
    //    clusters[dist_ind].rgb.val[2],0);

  }} // xy loop throug input image

  setImage("out", out);

  //setSignal("num", nc.size());
  /// update cluster_centers
  for (int k = 0; k < nc.size(); k++) {
    if (nc[k].numpix > 0) {
      nc[k].x = (int)  (nc[k].aggr_x/nc[k].numpix);
      nc[k].y = (int)  (nc[k].aggr_y/nc[k].numpix);
      nc[k].rgb = cv::Vec4b(
          (unsigned char) (nc[k].aggr_r/nc[k].numpix),
          (unsigned char) (nc[k].aggr_g/nc[k].numpix),
          (unsigned char) (nc[k].aggr_b/nc[k].numpix),
          0
          );
    }

    setSignal("x" + boost::lexical_cast<std::string>(k), nc[k].x);
    setSignal("y" + boost::lexical_cast<std::string>(k), nc[k].y);
    if (false) {
    setSignal("mnx" + boost::lexical_cast<std::string>(k), nc[k].min_x);
    setSignal("mny" + boost::lexical_cast<std::string>(k), nc[k].min_y);
    setSignal("mxx" + boost::lexical_cast<std::string>(k), nc[k].max_x);
    setSignal("mxy" + boost::lexical_cast<std::string>(k), nc[k].max_y);
    }
    setSignal("r" + boost::lexical_cast<std::string>(k), nc[k].rgb.val[0]);
    setSignal("g" + boost::lexical_cast<std::string>(k), nc[k].rgb.val[1]);
    setSignal("b" + boost::lexical_cast<std::string>(k), nc[k].rgb.val[2]);
    setSignal("p" + boost::lexical_cast<std::string>(k), nc[k].numpix);

  }

  clusters = nc;
    
  //setSignal("time", t1.elapsed());
}


PyrMean::PyrMean()
{
  cv::Mat in;
  setImage("in", in);
  setSignal("sp", 30);
  setSignal("sr", 30);
  setSignal("max_level", 2);
  // TBD TERMCRIT
  setSignal("term", 2);
}

bool PyrMean::update()
{
  const bool rv = Node::update();
  if (!rv) return false;
  
  cv::Mat in = getImage("in");
  if (in.empty()) return true;

  cv::Mat out_3;

  cv::Mat in_3 = cv::Mat(in.size(), CV_8UC3, cv::Scalar(0));  
  // just calling reshape(4) doesn't do the channel reassignment like this does
  int ch[] = {0,0, 1,1, 2,2};
  cv::mixChannels(&in, 1, &in_3, 1, ch, 3 );

  int max_level = getSignal("max_level");
  if (max_level > 4) { max_level = 4; setSignal("max_level", max_level); }
  if (max_level < 0) { max_level = 0; setSignal("max_level", max_level); }
  
  cv::pyrMeanShiftFiltering(in_3, out_3, 
      getSignal("sp"), getSignal("sr"),
      max_level,
      cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, getSignal("term"), 5)
      );

  cv::Mat out = cv::Mat(out_3.size(), CV_8UC4, cv::Scalar(0));  
  cv::mixChannels(&out_3, 1, &out, 1, ch, 3 );

  setImage("out", out);

}

} //bm
