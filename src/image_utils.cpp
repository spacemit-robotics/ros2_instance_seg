#include "instance_seg/image_utils.h"

#include <climits>
#include <cstring>

#include <opencv2/imgproc.hpp>

namespace instance_seg {
cv::Mat ImageMsgToBgr(const sensor_msgs::msg::Image& msg) {
    if (msg.height == 0 || msg.width == 0 || msg.height > INT_MAX ||
        msg.width > INT_MAX || msg.data.empty()) {
        return {};
    }
    const int type = (msg.encoding == "mono8") ? CV_8UC1
        : (msg.encoding == "bgr8") ? CV_8UC3
        : (msg.encoding == "rgb8") ? CV_8UC3
        : -1;
    if (type < 0) return {};
    const size_t minimum_step = static_cast<size_t>(msg.width) * CV_ELEM_SIZE(type);
    if (msg.step < minimum_step ||
        msg.data.size() < static_cast<size_t>(msg.step) * msg.height) {
        return {};
    }
    cv::Mat input(
        static_cast<int>(msg.height),
        static_cast<int>(msg.width),
        type,
        const_cast<uint8_t*>(msg.data.data()),
        msg.step);
    cv::Mat bgr;
    if (msg.encoding == "rgb8") cv::cvtColor(input, bgr, cv::COLOR_RGB2BGR);
    else if (msg.encoding == "mono8") cv::cvtColor(input, bgr, cv::COLOR_GRAY2BGR);
    else bgr = input.clone();
    return bgr;
}

sensor_msgs::msg::Image MatToImageMsg(
    const cv::Mat& image,
    const std_msgs::msg::Header& header,
    const std::string& encoding) {
    sensor_msgs::msg::Image out;
    out.header = header;
    out.height = static_cast<uint32_t>(image.rows);
    out.width = static_cast<uint32_t>(image.cols);
    out.encoding = encoding;
    out.is_bigendian = false;
    out.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
    out.data.resize(static_cast<size_t>(out.step) * out.height);
    const size_t row_bytes = static_cast<size_t>(image.cols) * image.elemSize();
    for (int row = 0; row < image.rows; ++row) {
        std::memcpy(
            out.data.data() + static_cast<size_t>(row) * out.step,
            image.ptr(row),
            row_bytes);
    }
    return out;
}

bool CaptureCameraFrame(
    cv::VideoCapture& cap,
    sensor_msgs::msg::Image& out_msg,
    const std_msgs::msg::Header& header) {
    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) return false;
    out_msg = MatToImageMsg(frame, header, "bgr8");
    return true;
}
}  // namespace instance_seg
