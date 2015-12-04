#ifndef OUTPUT_H
#define OUTPUT_H

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>

#include "nodes.h"

namespace bm
{
// TBD allow multiple?
class Output : public ImageNode
{
  int x, y, w, h;
  int seq_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_;
  // image_transport::CameraPublisher pub_;
  // boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

public:
  explicit Output(const std::string name);
  ~Output();
  virtual void init();

  bool setup(const int width, const int height);

  virtual bool update();
  virtual bool draw(cv::Point2f ui_offset);
};
}  // namespace bm

#endif  // OUTPUT_H
