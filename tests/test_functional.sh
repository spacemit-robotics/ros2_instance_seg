#!/bin/bash
# Dev-board functional test, adapted from yolo_general.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_TESTS="${SCRIPT_DIR}/../../yolo_general/tests"
# shellcheck disable=SC1091
source "${SHARED_TESTS}/common_setup.sh"
CONFIG="$(pkg_share_config instance_seg config/yolov8_seg.yaml)"
test -f "${CONFIG}"
MODEL_PATH="$(sed -n 's/^model_path:[[:space:]]*//p' "${CONFIG}" | head -1)"
MODEL_PATH="${MODEL_PATH/#\~/$HOME}"
ros2 run instance_seg instance_seg_node --ros-args -p "config_path:=${CONFIG}" \
  -p lazy_load:=false -p use_camera:=false -p image_topic:=/test/image_raw &
PID=$!
cleanup() {
  for echo_pid in "${MASK_ECHO:-}" "${INSTANCES_ECHO:-}" "${DEBUG_ECHO:-}"; do
    test -z "${echo_pid}" || kill "${echo_pid}" 2>/dev/null || true
  done
  stop_node_gracefully "${PUBLISHER_PID:-}"
  stop_node_gracefully "${PID}"
}
trap cleanup EXIT
assert_node_starts instance_seg_node
assert_topic_exists /perception/instance_segmentation
assert_topic_exists /instance_seg/instances
assert_topic_exists /instance_seg/debug_image

# A model-less board can still verify startup. With model weights present,
# actual mask, metadata, and debug outputs are mandatory after test input.
if test ! -f "${MODEL_PATH}"; then
  echo "SKIP inference assertions: model not found at ${MODEL_PATH}"
  exit 0
fi
MASK_FILE="$(mktemp)"; INSTANCES_FILE="$(mktemp)"; DEBUG_FILE="$(mktemp)"
trap 'rm -f "${MASK_FILE}" "${INSTANCES_FILE}" "${DEBUG_FILE}"; cleanup' EXIT
SENSOR_QOS=(--qos-reliability best_effort --qos-durability volatile)
timeout 45s ros2 topic echo /perception/instance_segmentation --once "${SENSOR_QOS[@]}" \
  >"${MASK_FILE}" 2>/dev/null & MASK_ECHO=$!
timeout 45s ros2 topic echo /instance_seg/instances --once >"${INSTANCES_FILE}" 2>/dev/null & INSTANCES_ECHO=$!
timeout 45s ros2 topic echo /instance_seg/debug_image --once "${SENSOR_QOS[@]}" \
  >"${DEBUG_FILE}" 2>/dev/null & DEBUG_ECHO=$!
sleep 1
TEST_IMAGE="$(grep '^test_image:' "${CONFIG}" | head -1 | sed 's/^test_image:[[:space:]]*//' | sed "s#^~#${HOME}#")"
PUBLISH_ARGS=(--topic /test/image_raw --count 40)
if [ -f "${TEST_IMAGE}" ]; then PUBLISH_ARGS+=(--image "${TEST_IMAGE}" --strict-image); fi
python3 "${SHARED_TESTS}/publish_test_image.py" "${PUBLISH_ARGS[@]}" & PUBLISHER_PID=$!
wait "${MASK_ECHO}"; wait "${INSTANCES_ECHO}"; wait "${DEBUG_ECHO}"
wait "${PUBLISHER_PID}"; PUBLISHER_PID=""
test -s "${MASK_FILE}" && test -s "${INSTANCES_FILE}" && test -s "${DEBUG_FILE}"
grep -Fq 'encoding: mono16' "${MASK_FILE}"
grep -Fq 'data:' "${INSTANCES_FILE}"
grep -Fq 'encoding: bgr8' "${DEBUG_FILE}"
echo 'instance_seg functional tests passed'
