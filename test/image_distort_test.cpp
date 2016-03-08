#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

// load an image, publish a camera_info
// which are published into a node graph that distorts the image, then
// rectifies it.  Then compare the rectified image with the original,
// pass the test if they aren't too different.
class ImageDistortTest : public testing::Test
{
protected:
  virtual void SetUp()
  {
    // publish to image and camera_info, subscribe to image_rect
    ros::param::get("~image", image_name_);

    // TODO(lwalter) want to have an it_ member, but how to initialize it?
    image_transport::ImageTransport it(nh_);
  }

  ros::NodeHandle nh_;
  std::string image_name_;
  bool has_new_image_;
  cv::Mat sent_image_;
  cv::Mat received_image_;
  image_transport::CameraPublisher cam_pub_;
  image_transport::Subscriber cam_sub_;
};

TEST_F(ImageDistortTest, rectifyTest)
{
  double diff = 0.0;
  EXPECT_LT(diff, 1000.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_distort_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
