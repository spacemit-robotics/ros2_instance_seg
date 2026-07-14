#!/bin/bash
# Contract test for the instance-segmentation ROS 2 node.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NODE="${ROOT}/src/instance_seg_node.cpp"
CONFIG="${ROOT}/config/yolov8_seg.yaml"

test -f "${NODE}"
test -f "${CONFIG}"
grep -Fq '#include <functional>' "${NODE}"
grep -Fq '!std::isfinite(camera_fps_) || camera_fps_ <= 0.0' "${NODE}"
grep -Fq 'camera_fps must be finite and greater than 0' "${NODE}"
grep -Fq 'seg->mask->type() != CV_8UC1' "${NODE}"
grep -Fq 'for (int row = 0; row < image.rows; ++row)' "${ROOT}/src/image_utils.cpp"
grep -Fq 'out.data.resize(static_cast<size_t>(out.step) * out.height)' "${ROOT}/src/image_utils.cpp"
grep -Fq 'msg.width > INT_MAX' "${ROOT}/src/image_utils.cpp"
grep -Fq 'const size_t minimum_step' "${ROOT}/src/image_utils.cpp"
grep -Fq 'service_->Infer(bgr, &response)' "${NODE}"
grep -Fq 'std::get_if<vision::Segmentation>' "${NODE}"
grep -Fq '"mono16"' "${NODE}"
grep -Fq 'service_->Draw(bgr, response, &out_image)' "${NODE}"
grep -Fq 'out_image.type() == CV_8UC3' "${NODE}"
grep -Fq 'std::max(1, static_cast<int>(1000.0 / camera_fps_))' "${NODE}"
grep -Fq 'instances_topic' "${NODE}"
grep -Fq 'mask_topic' "${NODE}"
grep -Fq 'class: deploy.yolov8_seg.YOLOv8SegDetector' "${CONFIG}"
grep -Fq '/instance_seg/debug_image' "${ROOT}/tests/test_functional.sh"
# Intentional: assert the functional script still embeds these literal strings.
# shellcheck disable=SC2016
grep -Fq 'SHARED_TESTS="${SCRIPT_DIR}/../../yolo_general/tests"' "${ROOT}/tests/test_functional.sh"
# shellcheck disable=SC2016
grep -Fq 'source "${SHARED_TESTS}/common_setup.sh"' "${ROOT}/tests/test_functional.sh"
# shellcheck disable=SC2016
grep -Fq '"${SHARED_TESTS}/publish_test_image.py"' "${ROOT}/tests/test_functional.sh"
test ! -e "${ROOT}/tests/common_setup.sh"
test ! -e "${ROOT}/tests/common_test_utils.sh"
test ! -e "${ROOT}/tests/publish_test_image.py"
test ! -e "${ROOT}/tests/test_config_valid.yaml"
test ! -e "${ROOT}/tests/test_performance.sh"
