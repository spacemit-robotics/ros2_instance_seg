#!/bin/bash
# Dev-board invalid-input test, adapted from yolo_general.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_TESTS="${SCRIPT_DIR}/../../yolo_general/tests"
# shellcheck disable=SC1091
source "${SHARED_TESTS}/common_setup.sh"
NODE="ros2 run instance_seg instance_seg_node --ros-args -p use_camera:=false"
INVALID="${SCRIPT_DIR}/test_config_invalid.yaml"
assert_node_fails_with_error "${NODE} -p score_threshold:=-0.1 -p lazy_load:=true" 'score_threshold.*range'
assert_node_fails_with_error "${NODE} -p score_threshold:=1.1 -p lazy_load:=true" 'score_threshold.*range'
assert_node_fails_with_error "${NODE} -p camera_fps:=0.0 -p lazy_load:=true" 'camera_fps.*finite.*greater than 0'
assert_node_fails_with_error "${NODE} -p camera_fps:=-1.0 -p lazy_load:=true" 'camera_fps.*finite.*greater than 0'
assert_node_fails_with_error "${NODE} -p config_path:=${INVALID} -p lazy_load:=false" 'YAML|parse|VisionService::Create failed|Exception'
echo 'instance_seg invalid-input tests passed'
