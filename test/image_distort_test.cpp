#include <cv_bridge/cv_bridge.h>
#include <gtest/gtest.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

// load an image, publish a camera_info
// which are published into a node graph that distorts the image, then
// rectifies it.  Then compare the rectified image with the original,
// pass the test if they aren't too different.
class ImageDistortTest : public testing::Test
{
public:

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_FATAL("cv_bridge exception: %s", e.what());
      return;
    }
    received_image_ = cv_ptr->image.clone();
    has_new_image_ = true;
  }

protected:
  virtual void SetUp()
  {
    has_new_image_ = false;
    // publish to image and camera_info, subscribe to image_rect
    ros::param::get("~image", image_name_);
    sent_image_ = cv::imread(image_name_);
    raw_image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8",
                                    sent_image_).toImageMsg();
    ROS_INFO_STREAM("loaded " << image_name_);
    std::cout << "loaded " << image_name_ << std::endl;
    // TODO(lucasw) also generate a pure white mask image to send through the pipeline

    // TODO(lucasw) want to have an it_ member, but how to initialize it?
    image_transport::ImageTransport it(nh_);
    cam_pub_ = it.advertiseCamera("image", 1);

    const float cx = sent_image_.cols/2;
    const float cy = sent_image_.rows/2;
   // Taken from vision_opencv/image_geometry/test/utest.cpp
    double D[] = {-0.363528858080088, 0.16117037733986861, -8.1109585007538829e-05, -0.00044776712298447841, 0.0};
    double K[] = {430.15433020105519,                0.0, cx,
                                 0.0, 430.60920415473657, cy,
                                 0.0,                0.0,                1.0};
    // TODO(lucasw) distort does not handle non-identy rectifications
    // how to undo it?
    double R[] = {1, 0, 0,
                  0, 1, 0,
                  0, 0, 1};
    double P[] = {K[0], 0.0,  K[2], 0.0,
                  0.0,  K[4], K[5], 0.0,
                  0.0,  0.0,  1.0,  0.0};

    cam_info_.header.frame_id = "tf_frame";
    cam_info_.height = sent_image_.rows;
    cam_info_.width  = sent_image_.cols;
    // No ROI
    cam_info_.D.resize(5);
    std::copy(D, D+5, cam_info_.D.begin());
    std::copy(K, K+9, cam_info_.K.begin());
    std::copy(R, R+9, cam_info_.R.begin());
    std::copy(P, P+12, cam_info_.P.begin());
    cam_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  }

  void publishRaw()
  {
    has_new_image_ = false;
    cam_pub_.publish(*raw_image_, cam_info_);
  }

  ros::NodeHandle nh_;
  std::string image_name_;
  sensor_msgs::ImagePtr raw_image_;
  bool has_new_image_;
  cv::Mat sent_image_;
  cv::Mat received_image_;
  sensor_msgs::CameraInfo cam_info_;
  image_transport::CameraPublisher cam_pub_;
  image_transport::Subscriber cam_sub_;
};

TEST_F(ImageDistortTest, rectifyTest)
{
  // TODO(lucasw) is there a way to create it and keep it in the class?
  image_transport::ImageTransport it(nh_);
  cam_sub_ = it.subscribe("image_rect",
                          1, &ImageDistortTest::imageCallback,
                            // this);
                            dynamic_cast<ImageDistortTest*>(this));
  // Wait for image_proc to be operational
  bool wait_for_topic = true;
  while (wait_for_topic)
  {
    // @todo this fails without the additional 0.5 second sleep after the
    // publisher comes online, which means on a slower or more heavily
    // loaded system it may take longer than 0.5 seconds, and the test
    // would hang until the timeout is reached and fail.
    if (cam_sub_.getNumPublishers() > 0)
       wait_for_topic = false;
    ros::Duration(0.5).sleep();
  }

  publishRaw();
  while(!has_new_image_)
  {
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
  // cv::imshow("sent", sent_image_);
  // cv::imshow("received", received_image_);
  // cv::waitKey(0);
  double diff = cv::norm(sent_image_, received_image_, cv::NORM_L1);
  // cv::Mat diff_image;
  // cv::absdiff(received_image_,sent_image_, diff_image);
  // cv::imshow("received", diff_image);
  // cv::waitKey(0);
  ROS_INFO_STREAM(diff);
  EXPECT_LT(diff, sent_image_.cols * sent_image_.rows * 255 * 0.05);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_distort_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
