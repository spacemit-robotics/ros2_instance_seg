#ifndef INSTANCE_UTILS_H
#define INSTANCE_UTILS_H

#include <vector>

#include "std_msgs/msg/float32_multi_array.hpp"
#include "vision_service.h"

namespace instance_seg {
std_msgs::msg::Float32MultiArray EncodeInstances(
    const std::vector<vision::Segmentation>& instances);
}  // namespace instance_seg

#endif  // INSTANCE_UTILS_H
