# instance_seg

## 项目简介

ROS2 实例分割节点，基于 `vision_service.h` 调用 `model_zoo/vision` 的 YOLOv8-Seg 模型，实现实时实例分割。

## 功能特性

- 支持 YOLOv8-Seg 实例分割
- 输出 `mono16` 实例 ID 掩码及实例元数据
- 提供可视化调试图像
- 支持直连摄像头或订阅图像话题
- 不支持：目标跟踪、语义分割

## 快速开始

### 环境准备

- ROS2 Humble 或更高版本
- 已编译的 `components/model_zoo/vision` 组件
- YOLOv8-Seg 模型文件

### 构建编译

```bash
colcon build --packages-select instance_seg
source install/setup.bash
```

### 运行示例

```bash
ros2 launch instance_seg instance_seg.launch.py
```

## 详细使用

### 依赖

- `components/model_zoo/vision`：提供 `libvision.so` 与 `vision_service.h`
- YOLOv8-Seg 模型及对应 yaml 配置

### 话题

| 类型 | 话题（默认） | 说明 |
|------|--------------|------|
| 订阅/发布 | `/camera/image_raw` | `use_camera=true` 时发布，否则订阅 |
| 发布 | `/perception/instance_segmentation` | `sensor_msgs/Image`，`mono16`；0 为背景，1..N 为本帧实例 ID |
| 发布 | `/instance_seg/instances` | `Float32MultiArray`，每实例 7 个数：id,label,score,x1,y1,x2,y2 |
| 发布 | `/instance_seg/debug_image` | 实例分割可视化图 |

### 配置

- 节点参数：`config/instance_seg.yaml`
- 默认模型配置：`config/yolov8_seg.yaml`
- 常用参数：`score_threshold`、`use_camera`、`camera_id`、`camera_fps`、`image_topic`
- 当 `use_camera=false` 时，从 `image_topic` 订阅图像；`camera_fps` 必须大于 0。

## 常见问题

- 若没有分割结果，先确认模型路径、输入图像话题和 `score_threshold`。
- 实例掩码中重叠区域以较后的有效实例为准；空、非单通道或尺寸不符的掩码会被忽略。
- 若从外部话题送图，请确认 `use_camera:=false`。

## 版本与发布

当前版本：1.0.0

变更记录：
- 初始版本发布

## 贡献方式

欢迎提交 Issue 和 Pull Request。

贡献者与维护者名单见：`CONTRIBUTORS.md`（如有）。

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
