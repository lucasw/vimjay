#include <opencv2/opencv.hpp>

void initDistortMap(const cv::Mat& cameraMatrix, const cv::Mat distCoeffs,
    const cv::Size size,
    cv::Mat& map1, cv::Mat& map2);

void distort(const cv::Mat& src, cv::Mat& image_dst,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
