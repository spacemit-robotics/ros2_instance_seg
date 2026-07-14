/** ROS Image/OpenCV conversion helpers. */
#ifndef IMAGE_UTILS_H
#define IMAGE_UTILS_H

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace instance_seg {
cv::Mat ImageMsgToBgr(const sensor_msgs::msg::Image& msg);
sensor_msgs::msg::Image MatToImageMsg(
    const cv::Mat& image,
    const std_msgs::msg::Header& header,
    const std::string& encoding);
bool CaptureCameraFrame(
    cv::VideoCapture& cap,
    sensor_msgs::msg::Image& out_msg,
    const std_msgs::msg::Header& header);
}  // namespace instance_seg

#endif  // IMAGE_UTILS_H
