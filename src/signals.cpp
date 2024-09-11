/*

  Copyright 2012 Lucas Walter

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
 */

#include "vimjay/signals.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/thread.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/console.h>

#include <deque>

#include "vimjay/config.h"

extern "C" {
#include <dso_noises/noise1234.h>
#include <dso_noises/simplexnoise1234.h>
#include <dso_noises/sdnoise1234.h>
// #include <dso_noises/srdnoise23.h>
}

using namespace cv;
using namespace std;

namespace bm
{
//////////////////////////////////////////////////
MiscSignal::MiscSignal(const std::string name) : Signal(name)
{
}

void MiscSignal::init()
{
  Signal::init();
  vcol = cv::Scalar(0, 90, 255);
  setSignal("mode", 7, false, ROLL, 0, 7);
}

// void MiscSignal::setup(const float new_step, const float offset, const float min, const float max)
// {
//   Signal::setup(new_step, offset, min, max);
//}

bool MiscSignal::handleKey(int key)
{
  bool valid_key = Signal::handleKey(key);
  if (valid_key) return true;

  // valid_key = true;

  // TBD
  if (valid_key) setDirty();

  return valid_key;
}

bool MiscSignal::update()
{
  // don't call Signal::update because it will contradict this update
  if (!Node::update()) return false;

  float step = getSignal("step");

  // TBD use step for the random number seed
  int new_state = step;
  // TBD step changes a lot, this may not be that ideal
  if (new_state != state)
  {
    ROS_INFO_STREAM(name << " " << state << " " << step << " new state " << 0xFFFFFFF - (int)state);
    state = step; // 0xFFFFFFF - (int)step;
    rng = cv::RNG(0xFFFFFFF - (int)state);
  }

  float min = getSignal("min");
  float max = getSignal("max");
  float value = getSignal("value");
  float t = getSignal("t");

  if (max < min)
  {
    max = min;
    setSignal("max", max);
  }

  string port = "mode";
  int mode = getSignal(port);

  boost::shared_ptr<Connector> con;
  string src_port;
  getInputPort(SIGNAL, port, con, src_port);

  float span = max - min;

  if (mode == 0)
  {
    // TBD need setDescription(signal_name, type, desc) function
    con->description = "triangle";
    value += step;
    if (value > max)
    {
      step = -fabs(step);
      value = max;
    }
    if (value < min)
    {
      step = fabs(step);
      value = min;
    }

  }
  else if (mode == 1)
  {
    con->description = "sawtooth";
    value += step;
    if (span == 0)
    {
      value = min;
    }
    else
    {
      while (value > max)
      {
        value -= (span);
      }
      while (value < min)
      {
        value += (span);
      }
    }

  }
  else if (mode == 2)
  {
    con->description = "ramp_saturate";
    value += step;
    if (value > max) value = max;
    if (value < min) value = min;

  }
  else if (mode == 3)
  {
    con->description = "toggle";
    if (value < max)
    {
      value = max;
    }
    if (value > min)
    {
      value = min;
    }

  }
  else if (mode == 4)
  {
    con->description = "rnd_uniform";
    value = rng.uniform(min, max);

  }
  else if (mode == 5)
  {
    con->description = "rnd_gaussian";
    const float rnd = rng.gaussian(max - min);
    value = rnd + min;
  }
  // TBD noise(x), snoise(x), sdnoise(x)
  else if (mode == 6)
  {
    con->description = "perlin";
    value = min + (max - min) / 2 + noise1(t) * (max - min) / 2;
  }
  else if (mode == 7)
  {
    con->description = "simplex";
    value = min + (max - min) / 2 + snoise1(t) * (max - min) / 2;
  }

  t += abs(step);
  setSignal("t", t);
  setSignal("step", step);
  setSignal("value", value);

  setDirty();

  ROS_DEBUG_STREAM_COND(log_level > 3, "Signal " << name << " " << value);
  return true;
}

// TBD need control to set value to max or min

///////////////////////////////////////////////////
Trig::Trig(const string name) :
  Node(name)
{
}

void Trig::init()
{
  Node::init();
  setSignal("in", 0);
  setSignal("radius", 1);
  setSignal("rad_deg_nrm", 2);
  setSignal("cos", 0, true);
  setSignal("sin", 0, true);
  setSignal("tan", 0, true);
}

bool Trig::update()
{
  if (!Node::update()) return false;

  const int rad_deg_nrm = getSignal("rad_deg_nrm");
  float in = getSignal("in");
  const float radius = getSignal("radius");
  // convert deg to radians
  if (rad_deg_nrm == 1) in *= M_PI / 180.0;
  // convert from normalized radians to radians
  if (rad_deg_nrm == 2) in *= M_PI;

  setSignal("cos", radius * cos(in));
  setSignal("sin", radius * sin(in));
  setSignal("tan", radius * tan(in));

  return true;
}
/////////////////////////////////////////////////////////////////////////////

/// TBD It would be useful if all signals had buffers, it wouldn't cost
/// much and would add a lot of convenience.
SigBuffer::SigBuffer(const std::string name) :
  ImageNode(name)
{
}

void SigBuffer::init()
{
  ImageNode::init();
  setSignal("in", 0);

  ROS_DEBUG_STREAM_COND(log_level > 1, " setting out image");
  cv::Mat tmp;
  cv::Size sz = Config::inst()->getImSize();
  tmp = cv::Mat(sz, MAT_FORMAT_C3, cv::Scalar(0));
  setImage("out", tmp);
  setSignal("out", 0);
  setSigBuf("out", true);
  // 0 - buffer size index input
  setSignal("ind", 0);
  // 0 - 1.0 fractional input
  setSignal("fr", 0);

  setSignal("min", 0);
  setSignal("max", 0);
  setSignal("max_size", 2000);
  setSignal("cur_size", 0);

  setSignal("middle", 0.5);
  setSignal("scale", 1.0);
  setSignal("scale_mode", 1, false, ROLL, 0, 2);
  setSignal("sample_mode", 1, false, ROLL, 0, 1);
}

bool SigBuffer::setOut()
{
  if (sigs.size() <= 0) return false;

  // TBD also provide a 0-1.0 input, take whichever is dirty here
  bool b1, ind_is_dirty, fr_is_dirty;
  int ind = (int)getSignal("ind", b1, ind_is_dirty);
  float fr = getSignal("fr", b1, fr_is_dirty);
  float val = 0;
  // want to keeping getting with last dirty input
  if (fr_is_dirty || (!ind_is_dirty && last_get_was_fr))
  {
    val = getFr(fr);
    last_get_was_fr = true;
  }
  else
  {
    val = getInd(ind);
    last_get_was_fr = false;
  }
  setSignal("out", val);
  return true;
}

bool SigBuffer::update()
{
  const bool rv = Node::update();
  if (!rv) return false;

  int max_size = getSignal("max_size");
  if (max_size < 0) max_size = 0;

  // TBD optionally sample all the time
  bool b1, con_is_dirty;
  const float new_sig = getSignal("in", b1, con_is_dirty);
  if (con_is_dirty || getBool("sample_mode"))
  {
    boost::mutex::scoped_lock l(sigs_mutex);
    sigs.push_back(new_sig);

    while (sigs.size() > max_size) sigs.pop_front();

    setSignal("cur_size", sigs.size());
  }

  setOut();
  // setSignal("out", sigs[0]);


  return true;
}

bool SigBuffer::draw(cv::Point2f ui_offset)
{
  cv::Mat vis = getImage("out");
  if (vis.empty())
  {
    ROS_WARN_STREAM("out is empty");
    cv::Size sz = Config::inst()->getImSize();
    vis = cv::Mat(sz, MAT_FORMAT_C3, cv::Scalar(0));
  }

  // update the vis image- TBD add option to disable this?
  if (isDirty(this, 24))
  {

    // TBD error check the mat
    vis = cv::Scalar(0);

    float new_min =  1e6;
    float new_max = -1e6;
    const float min = getSignal("min");
    const float max = getSignal("max");

    // TBD only do this if scale_mode == 0
    const int scale_mode = getSignal("scale_mode");
    float middle = (max + min) / 2.0;
    // the 2.1 creates a little padding
    float scale = fabs(max - min);
    if (scale_mode == 1)
    {
      middle = getSignal("middle");
      scale = getSignal("scale");
    }
    else
    {
      setSignal("middle", middle);
      setSignal("scale", scale);
    }

    float div = (float)sigs.size() / (float)vis.cols;

    // this causes signal values to be skipped
    // if there are more than the number of column pixels
    // This maybe should be optional, it is nice to see the extent
    // of the higher frequency signal even if it can't be
    // completely resolved.
    int inc = 1;
    if (div > 1.0) inc = (int) div;

    ROS_DEBUG_STREAM_COND(log_level > 5, sigs.size() << ":" << div << " " << inc << " "
                          << scale << " " << middle);

    // TBD should we always redraw?  Should we
    {

      {
        // TBD make this optional
        int ind = int(getSignal("ind"));
        const float val = getInd(ind);
        const float y1 = vis.rows - vis.rows * (0.5 + (val  - middle) / scale);
        cv::line(vis,
                 cv::Point2f((float)(ind) / div,     y1),
                 cv::Point2f((float)(ind) / div, vis.rows),
                 cv::Scalar(255, 0, 0),
                 1);
      }

      boost::mutex::scoped_lock l(sigs_mutex);

      for (int i = 0; i < (int)sigs.size() - inc; i += inc)
      {
        const float val = sigs[i];
        const float val2 = sigs[i + inc];
        if (val > new_max) new_max = val;
        if (val < new_min) new_min = val;

        const float y1 = vis.rows - vis.rows * (0.5 + (val  - middle) / scale);
        const float y2 = vis.rows - vis.rows * (0.5 + (val2 - middle) / scale);

        cv::line(vis,
                 cv::Point2f((float)(i) / div,     y1),
                 cv::Point2f((float)(i + 1) / div, y2),
                 cv::Scalar(255, 255, 255),
                 1,
                 4);
      }
    }

    // an external signal will override, but no way for manual input to override
    setSignal("min", new_min);
    setSignal("max", new_max);

    setImage("out", vis);
  }

  return ImageNode::draw(ui_offset);
}

// bool SigBuffer::writeSignals();

// not the same as the inherited get on purpose
// many callers per time step could be calling this
float SigBuffer::getFr(const float fr)
{
  int ind = (int)(fr * (float)sigs.size());
  // if (fr < 0) {
  //  ind = frames.size() - ind;
  // }

  // TBD set the signal ind to be this ind?
  const float val = getInd(ind);

  setSignal("ind", ind);

  return val;
}

float SigBuffer::getInd(int& ind)
{
  boost::mutex::scoped_lock l(sigs_mutex);
  if (sigs.size() < 1)
  {
    ROS_DEBUG_STREAM_COND(log_level > 1, "no sigs returning 0");
    return 0;
  }
  // if (ind > sigs.size() - 1) ind = sigs.size() - 1;
  // if (ind < 0) ind = 0;
  ind %= sigs.size();

  ROS_DEBUG_STREAM_COND(log_level > 2, name << " ind " << ind);

  // VLOG_EVERY_N(1,10)
  // LOG_EVERY_N(INFO, 10) << ind << " " << sigs.size();
  return sigs[ind];
}


SigBufferXY::SigBufferXY(const std::string name) :
  SigBuffer(name)
{
}

void SigBufferXY::init()
{
  SigBuffer::init();
  setSignal("set_ind", 0);
}

bool SigBufferXY::update()
{
  const bool rv = Node::update();
  if (!rv) return false;

  const int max_size = int(getSignal("max_size"));
  // TBD optionally sample all the time
  bool b1, con_is_dirty_1, con_is_dirty_2;
  int ind = getSignal("set_ind", b1, con_is_dirty_1);
  const float new_sig = getSignal("in", b1, con_is_dirty_2);
  if (con_is_dirty_1 || con_is_dirty_2)
  {
    boost::mutex::scoped_lock l(sigs_mutex);
    if (max_size != sigs.size())
      sigs.resize(max_size, 0);

    ind %= sigs.size();
    if (sigs.size() > 0)
      sigs[ind] = new_sig;

  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////
// Sig add is similar in structure to the image multiply,
// so it has pairs of signal inputs that are each multiplied together
// but then each set is summed
SigAdd::SigAdd(const std::string name) :
  Node(name)
{
}

void SigAdd::init()
{
  Node::init();
  setSignal("out", 0, true);
  setSignal("mula0", 1);
  setSignal("mulb0", 0);
}

bool SigAdd::update()
{
  if (!Node::update()) return false;

  if (!isDirty(this, 35))
  {
    ROS_DEBUG_STREAM_COND(log_level > 1, name << " not dirty ");
    return true;
  }

  float sum = 0;

  for (int i = 0 ; i < ports.size(); i++)
  {
    if (ports[i]->type != SIGNAL) continue;
    const string port_a = ports[i]->name;

    if (port_a.substr(0, 4) != "mula")
    {
      ROS_DEBUG_STREAM_COND(log_level > 2, name << " : " << port_a.substr(0, 4) << " " << port_a);
      continue;
    }

    const string port_b = "mulb" + port_a.substr(4);

    const float a = getSignal(port_a);
    const float b = getSignal(port_b);

    ROS_DEBUG_STREAM_COND(log_level > 2, name <<  " " << CLTXT << port_a << " " << port_b << " "
                          << CLVAL << sum << " + " << a << " * " << b << CLNRM);
    sum += getSignal(port_a) * getSignal(port_b);

  }
  ROS_DEBUG_STREAM_COND(log_level > 2, name << " " << sum);

  setSignal("out", sum);

  // TBD The dirtiness of the output signal takes care of this?
  setDirty();
  isDirty(this, 35);

  return true;

}

bool SigAdd::handleKey(int key)
{
  bool valid_key = Node::handleKey(key);
  if (valid_key) return true;

  valid_key = true;
  if (key == '[')
  {

    // add an input addition port, TBD move to function
    int add_num = 0;
    for (int i = 0; i < ports.size(); i++)
    {
      if (ports[i]->type != SIGNAL) continue;
      const string port = ports[i]->name;

      if (port.substr(0, 4) != "mula")
      {
        ROS_DEBUG_STREAM_COND(log_level > 1, name << " : " << port.substr(0, 4) << " " << port);
        continue;
      }
      add_num++;
    }

    // add a new addition port
    const string port_a = "mula" + boost::lexical_cast<string>(add_num);
    const string port_b = "mulb" + boost::lexical_cast<string>(add_num);
    setSignal(port_a, 1);
    setSignal(port_b, 0);

    // TBD make a way to delete a port
  }
  else
  {
    valid_key = false;
  }

  // TBD
  if (valid_key) setDirty();

  return valid_key;
}

//////////////////////
SigGreater::SigGreater(const std::string name) :
  Signal(name)
{
}

void SigGreater::init()
{
  Signal::init();
  // TBD more convenient to output all possibilities, or provide mode selector?
  setSignal("greater", 0, true);
  setSignal("less", 0, true);
  setSignal("equal", 0, true);
  setSignal("in0", 1);
  setSignal("in1", 0);
}

bool SigGreater::update()
{
  if (!Node::update()) return false;

  if (!isDirty(this, 35))
  {
    ROS_DEBUG_STREAM_COND(log_level > 1, name << " not dirty ");
    return true;
  }

  float in0 = getSignal("in0");
  float in1 = getSignal("in1");

  setSignal("greater", (in0 > in1) ? 1.0 : 0.0);
  setSignal("less", (in0 < in1) ? 1.0 : 0.0);
  setSignal("equal", (in0 == in1) ? 1.0 : 0.0);

  setDirty();

  return true;

}

Mean::Mean(const std::string name) :
  Signal(name)
{
}

void Mean::init()
{
  Signal::init();
  // TBD more convenient to output all possibilities, or provide mode selector?
  cv::Mat tmp;
  setImage("in", tmp);
  const bool internally_set = true;
  const float initial_val = 0.0;
  setSignal("mean",   initial_val, internally_set);
  setSignal("mean_r", initial_val, internally_set);
  setSignal("mean_g", initial_val, internally_set);
  setSignal("mean_b", initial_val, internally_set);
  // TBD stddev  setSignal("equal",initial_val, internally_set);
}

bool Mean::update()
{
  if (!Node::update()) return false;

  if (!isDirty(this, 35))
  {
    ROS_DEBUG_STREAM_COND(log_level > 1, name << " not dirty ");
    return true;
  }

  cv::Mat in = getImage("in");

  cv::Scalar mean = cv::mean(in); // TBD optional mask

  float b = mean.val[0];
  float g = mean.val[1];
  float r = mean.val[2];
  float mean_all = (b + g + r) / 3.0;

  setSignal("mean", mean_all);
  setSignal("mean_r", r);
  setSignal("mean_g", g);
  setSignal("mean_b", b);

  setDirty();

  return true;

}

/////////////////////////////////////////
SigADSR::SigADSR(const std::string name) :
  ImageNode(name)
{

}

enum adsrType
{
  LOW = 0,
  ATTACK,
  DECAY,
  SUSTAIN,
  RELEASE
};

void SigADSR::init()
{
  ImageNode::init();

  // attack is the per-update amount to add to the signal in the attack
  // phase.
  setSignal("attack",  0.001);
  // peak is actually a scale value, it doesn't change the character of
  // the other parameters
  setSignal("peak",    1.0);
  setSignal("decay",   0.0005);
  setSignal("sustain", 0.5);
  setSignal("release", 0.0002);

  const string sig = "in0";
  setSignal(sig, 0, false, 0.0, 1.0);
  const bool internally_set = true;
  // 5 different phase:
  // low, attack, decay, sustain, release
  setSignal("phase_" + sig, 0, internally_set, ROLL, LOW, RELEASE);
  // by setting internally set to false, it is possible
  // to keep updating the output and TBD it should stop once LOW
  // is reached
  setSignal("out_" + sig, 0); //, internally_set);
}

bool SigADSR::update()
{
  if (!Node::update()) return false;

  // TBD replace 35 with number that is a function
  // of the string name of this class?  But we
  // want it to be unique when the base class calls
  // it, and it will have the same type when the
  // base class update gets the type name.
  if (!isDirty(this, 35))
  {
    ROS_DEBUG_STREAM_COND(log_level > 1, name << " not dirty ");
    return true;
  }


  const float attack  = getSignal("attack");
  const float peak    = getSignal("peak");
  const float decay   = getSignal("decay");
  const float sustain = getSignal("sustain");
  const float release = getSignal("release");

  // TBD for loop around all inN inputs
  {
    const string sig = "in0";

    float val = getSignal(sig);
    if (val < 0) val = 0;

    const string sig_out = "out_" + sig;
    const string sig_phase = "phase_" + sig;

    float val_out = getSignal(sig_out);
    const int phase = getSignal(sig_phase);

    if (phase == LOW)
    {
      setSignal(sig_out,  0.0);

      if (val > 0.0)
      {
        setSignal(sig_phase, ATTACK);
      }

    }
    else if (phase == ATTACK)
    {
      // instead of or in addition to linear addition,
      // should have s-curve like addition also, or exponential,
      // or velocity/acceleration mode
      // or anything else that seems good
      val_out += attack * peak;
      // TBD if val is less than 1.0, should peak be modified
      // by that?
      // if (val_out > peak * val) {
      if (val <= 0)
      {
        setSignal(sig_phase, RELEASE);
      }
      else if (val_out > peak)
      {
        val_out = peak;
        setSignal(sig_phase, DECAY);
      }
      setSignal(sig_out, val_out);

    }
    else if (phase == DECAY)
    {

      val_out -= decay * peak;
      if (val <= 0)
      {
        setSignal(sig_phase, RELEASE);
      }
      else if (val_out < sustain)
      {
        val_out = sustain;
        setSignal(sig_phase, SUSTAIN);
      }
      setSignal(sig_out, val_out);

    }
    else if (phase == SUSTAIN)
    {

      if (val <= 0)
      {
        setSignal(sig_phase, RELEASE);
      }
      setSignal(sig_out, sustain);

    }
    else if (phase == RELEASE)
    {

      val_out -= release * peak;
      if (val_out < 0)   // TBD have low value also
      {
        val_out = 0;
        setSignal(sig_phase, LOW);
      }
      setSignal(sig_out, val_out);

    }

  } // loop around n inputs

  return true;
} // sig adsr update


SigToInd::SigToInd(const std::string name) :
  Signal(name)
{

}

void SigToInd::init()
{
  Signal::init();

  setSignal("out", 0);
  setSignal("in0", 0);
  setSignal("in1", 0);
  setSignal("in2", 0);
  setSignal("in3", 0);
}

bool getMatchingPorts(
  const std::string pattern,
  const int port_type,
  vector<boost::shared_ptr<Connector> >& ports,
  vector<boost::shared_ptr<Connector> >& matched_ports
)
{
  for (int i = 0; i < ports.size(); i++)
  {
    if (ports[i]->type != port_type) continue;
    const string port = ports[i]->name;
    if (port.substr(0, 2) != pattern)
    {
      ROS_DEBUG_STREAM_COND(log_level > 5, /*name <<*/ " : " << port.substr(0, 3) << " " << port);
      continue;
    }

    matched_ports.push_back(ports[i]);
  }

  return true;
}

bool SigToInd::update()
{
  if (!Node::update()) return false;

  if (!isDirty(this, 35))
  {
    return true;
  }

  vector<boost::shared_ptr<Connector> > matched_ports;
  getMatchingPorts("in", SIGNAL, ports, matched_ports);

  for (size_t i = 0; i < matched_ports.size(); i++)
  {
    bool b1 = false;
    bool is_dirty = false;
    // kind of needless, could get the signal straight from the port
    float val = getSignal(matched_ports[i]->name, b1, is_dirty);
    if ((val > 0) && is_dirty)
    {
      setSignal("out", i);
      ROS_INFO_STREAM(name << " " << matched_ports[i]->name << " " << val);
      break;
    }
  }
  return true;
}  // SigToInd::update
}  // namespace bm

