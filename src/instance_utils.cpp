#include "instance_seg/instance_utils.h"

namespace instance_seg {
std_msgs::msg::Float32MultiArray EncodeInstances(
    const std::vector<vision::Segmentation>& instances) {
    constexpr uint32_t kFields = 7;
    std_msgs::msg::Float32MultiArray out;
    out.layout.dim.resize(2);
    out.layout.dim[0].label = "num_instances";
    out.layout.dim[0].size = static_cast<uint32_t>(instances.size());
    out.layout.dim[0].stride = out.layout.dim[0].size * kFields;
    out.layout.dim[1].label = "fields(id,label,score,x1,y1,x2,y2)";
    out.layout.dim[1].size = kFields;
    out.layout.dim[1].stride = kFields;
    out.data.reserve(instances.size() * kFields);
    for (size_t i = 0; i < instances.size(); ++i) {
        const auto& item = instances[i];
        out.data.insert(out.data.end(), {static_cast<float>(i + 1),
            static_cast<float>(item.label), item.score, item.bbox.x1, item.bbox.y1,
            item.bbox.x2, item.bbox.y2});
    }
    return out;
}
}  // namespace instance_seg
