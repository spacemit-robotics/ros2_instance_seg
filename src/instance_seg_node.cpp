/** ROS 2 YOLOv8-Seg instance segmentation node. */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "instance_seg/image_utils.h"
#include "instance_seg/instance_utils.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "vision_service.h"

class InstanceSegNode : public rclcpp::Node {
public:
    InstanceSegNode() : Node("instance_seg_node") {
        std::string config_path = declare_parameter<std::string>("config_path", "");
        const bool lazy_load = declare_parameter<bool>("lazy_load", true);
        score_threshold_ = declare_parameter<double>("score_threshold", 0.25);
        if (!std::isfinite(score_threshold_) || score_threshold_ < 0.0 || score_threshold_ > 1.0) {
            throw std::runtime_error("score_threshold out of range, must be in [0, 1]");
        }
        image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
        mask_topic_ = declare_parameter<std::string>("mask_topic", "/perception/instance_segmentation");
        instances_topic_ = declare_parameter<std::string>("instances_topic", "/instance_seg/instances");
        debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/instance_seg/debug_image");
        use_camera_ = declare_parameter<bool>("use_camera", true);
        camera_id_ = declare_parameter<int>("camera_id", 0);
        camera_fps_ = declare_parameter<double>("camera_fps", 30.0);
        if (!std::isfinite(camera_fps_) || camera_fps_ <= 0.0) {
            throw std::runtime_error("camera_fps must be finite and greater than 0");
        }
        if (config_path.empty()) config_path = DefaultConfigPath();
        if (config_path.empty()) throw std::runtime_error("config_path is empty; set a YOLOv8-Seg yaml path");

        service_ = VisionService::Create(config_path, "", lazy_load);
        if (!service_) throw std::runtime_error("VisionService::Create failed: " + VisionService::LastCreateError());
        masks_pub_ = create_publisher<sensor_msgs::msg::Image>(mask_topic_, rclcpp::SensorDataQoS());
        instances_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(instances_topic_, 10);
        debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SensorDataQoS());

        if (use_camera_) {
            image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS());
            cap_.open(camera_id_);
            if (!cap_.isOpened()) throw std::runtime_error("instance_seg: cannot open camera id=" + std::to_string(camera_id_));
            const int period_ms = std::max(1, static_cast<int>(1000.0 / camera_fps_));
            camera_timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                std::bind(&InstanceSegNode::OnCameraTimer, this));
        } else {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(image_topic_, rclcpp::SensorDataQoS(),
                std::bind(&InstanceSegNode::OnImage, this, std::placeholders::_1));
        }
        RCLCPP_INFO(get_logger(), "instance_seg_node started, config=%s, input=%s", config_path.c_str(), image_topic_.c_str());
    }

private:
    static std::string DefaultConfigPath() {
        try { return ament_index_cpp::get_package_share_directory("instance_seg") + "/config/yolov8_seg.yaml"; }
        catch (...) { return ""; }
    }
    void OnCameraTimer() {
        std_msgs::msg::Header header;
        header.stamp = now(); header.frame_id = "camera";
        sensor_msgs::msg::Image image;
        if (!instance_seg::CaptureCameraFrame(cap_, image, header)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "camera read empty frame"); return;
        }
        image_pub_->publish(image);
        const cv::Mat bgr = instance_seg::ImageMsgToBgr(image);
        if (!bgr.empty()) ProcessFrame(header, bgr);
    }
    void OnImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        const cv::Mat bgr = instance_seg::ImageMsgToBgr(*msg);
        if (bgr.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "invalid input image: %s", msg->encoding.c_str()); return;
        }
        ProcessFrame(msg->header, bgr);
    }
    void ProcessFrame(const std_msgs::msg::Header& header, const cv::Mat& bgr) {
        VisionServiceResponse response;
        if (service_->Infer(bgr, &response) != VISION_SERVICE_OK || !response.ok) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "infer failed: %s", service_->LastError().c_str()); return;
        }
        cv::Mat ordinal_mask = cv::Mat::zeros(bgr.rows, bgr.cols, CV_16UC1);
        std::vector<vision::Segmentation> valid_instances;
        for (const auto& result : response.results) {
            const auto* seg = std::get_if<vision::Segmentation>(&result);
            if (seg == nullptr || seg->score < static_cast<float>(score_threshold_)) continue;
            if (!seg->mask || seg->mask->empty() || seg->mask->type() != CV_8UC1 ||
                seg->mask->rows != bgr.rows || seg->mask->cols != bgr.cols) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "skipping segmentation with null, non-CV_8UC1, invalid, or dimension-mismatched mask");
                continue;
            }
            if (valid_instances.size() >= 65535) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "too many instances for mono16 mask"); break;
            }
            cv::Mat binary;
            cv::compare(*seg->mask, 0, binary, cv::CMP_NE);
            ordinal_mask.setTo(static_cast<uint16_t>(valid_instances.size() + 1), binary);
            valid_instances.push_back(*seg);
        }
        masks_pub_->publish(instance_seg::MatToImageMsg(ordinal_mask, header, "mono16"));
        instances_pub_->publish(instance_seg::EncodeInstances(valid_instances));
        cv::Mat out_image;
        if (service_->Draw(bgr, response, &out_image) == VISION_SERVICE_OK && !out_image.empty() &&
            out_image.rows == bgr.rows && out_image.cols == bgr.cols &&
            out_image.type() == CV_8UC3) {
            debug_pub_->publish(instance_seg::MatToImageMsg(out_image, header, "bgr8"));
        }
    }
    std::unique_ptr<VisionService> service_;
    double score_threshold_{0.25};
    std::string image_topic_, mask_topic_, instances_topic_, debug_image_topic_;
    bool use_camera_{true}; int camera_id_{0}; double camera_fps_{30.0};
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr camera_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_, masks_pub_, debug_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr instances_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try { rclcpp::spin(std::make_shared<InstanceSegNode>()); }
    catch (const std::exception& e) { RCLCPP_ERROR(rclcpp::get_logger("instance_seg_node"), "Exception: %s", e.what()); rclcpp::shutdown(); return 1; }
    rclcpp::shutdown(); return 0;
}
