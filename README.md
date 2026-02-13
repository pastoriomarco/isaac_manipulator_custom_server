# isaac_manipulator_custom_server

Perception-centered extension package set for Isaac Manipulator.

This repository focuses on one specific capability: robustly retrieving
dynamic multi-object 6D poses from a scene (for example, bin picking setups),
with optional per-scan object selection when you want same-object-type
multi-instance results, and exposing them through a stable service interface
that other systems can call on demand.

Top-level packages:

- `isaac_manipulator_pose_server`
- `isaac_manipulator_server_interfaces`

## Scope

This repo provides:

- A persistent scan server that can be triggered repeatedly.
- A service API to rescan and return all currently detected object poses.
- Multi-object aggregation logic on top of Isaac Manipulator perception
  servers, with optional object-specific filtering per scan.
- Launch files for perception-only bringup and scan server bringup.
- Continuous orchestration interfaces for topic-driven control and tracked-object streaming:
  - `/isaac_manipulator_pose_server/pipeline_command`
  - `/isaac_manipulator_pose_server/tracked_objects`
  - `/isaac_manipulator_pose_server/pipeline_status`
  - `/isaac_manipulator_pose_server/mark_object_used`

This repo intentionally does **not** provide:

- Full manipulation orchestration (planning/execution/control loops).
- Robot driver bringup as a hard requirement for perception-only workflows.
- End-to-end pick-and-place behavior trees.

## How It Fits With Isaac Manipulator

`isaac_manipulator_custom_server` depends on Isaac Manipulator perception
components (`isaac_manipulator_servers`, interfaces, and FoundationPose/RT-DETR
pipeline pieces). It does not replace Isaac Manipulator; it wraps and
specializes it for reusable multi-object pose serving.

In short:

- Isaac Manipulator provides the core perception/action servers.
- This repo adds a clean, reusable "scan and return object poses" layer for
  downstream consumers (e.g., task planners or custom motion stacks).

## FoundationPose Quickstart Rosbag Test

This is the command sequence used to validate the custom scan server with the
FoundationPose quickstart rosbag (single-object scene).

### Terminal 1: Build and launch perception + scan server

```bash
cd "$ISAAC_ROS_WS"
source /opt/ros/jazzy/setup.bash

colcon build --packages-select \
  isaac_manipulator_server_interfaces \
  isaac_manipulator_pose_server

source install/setup.bash

ros2 launch isaac_manipulator_pose_server perception_scan_server.launch.py \
  camera_type:=ISAAC_SIM \
  use_sim_time:=true \
  foundationpose_depth_topic:=/foundation_pose_server/depth \
  rgb_image_topic:=/image_rect \
  rgb_camera_info_topic:=/camera_info_rect \
  depth_image_topic:=/depth \
  depth_camera_info_topic:=/camera_info_rect \
  rgb_image_width:=640 rgb_image_height:=480 \
  depth_image_width:=640 depth_image_height:=480 \
  input_qos:=DEFAULT output_qos:=DEFAULT \
  input_fps:=8 dropped_fps:=8 \
  rt_detr_confidence_threshold:=0.3 \
  rtdetr_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan" \
  refine_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan" \
  score_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/score_trt_engine.plan" \
  refine_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/refine_model.onnx" \
  score_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/score_model.onnx" \
  mesh_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_foundationpose/soup_can/soup_can.obj" \
  texture_path:="$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_foundationpose/soup_can/baked_mesh_tex0.png"
```

To switch detector frontend at launch, set:

- `object_detection_model:=RT_DETR` (default), or
- `object_detection_model:=YOLOV8` with the full command below.

```bash
ros2 launch isaac_manipulator_pose_server perception_scan_server.launch.py \
  camera_type:=ISAAC_SIM \
  use_sim_time:=true \
  foundationpose_depth_topic:=/foundation_pose_server/depth \
  rgb_image_topic:=/image_rect \
  rgb_camera_info_topic:=/camera_info_rect \
  depth_image_topic:=/depth \
  depth_camera_info_topic:=/camera_info_rect \
  rgb_image_width:=640 rgb_image_height:=480 \
  depth_image_width:=640 depth_image_height:=480 \
  input_qos:=DEFAULT output_qos:=DEFAULT \
  input_fps:=8 dropped_fps:=8 \
  object_detection_model:=YOLOV8 \
  yolov8_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/yolov8/your_model.onnx" \
  yolov8_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/yolov8/your_model.plan" \
  yolov8_input_tensor_names:='["input_tensor"]' \
  yolov8_input_binding_names:='["images"]' \
  yolov8_output_tensor_names:='["output_tensor"]' \
  yolov8_output_binding_names:='["output0"]' \
  yolov8_confidence_threshold:=0.25 \
  yolov8_nms_threshold:=0.45 \
  yolov8_num_classes:=1 \
  yolov8_network_image_width:=640 \
  yolov8_network_image_height:=640 \
  refine_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan" \
  score_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/score_trt_engine.plan" \
  refine_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/refine_model.onnx" \
  score_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/score_model.onnx" \
  mesh_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_foundationpose/soup_can/soup_can.obj" \
  texture_path:="$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_foundationpose/soup_can/baked_mesh_tex0.png"
```

Default runtime config file:

- `src/isaac_manipulator_custom_server/isaac_manipulator_pose_server/params/pose_server.yaml`

Pre-check required packages in your sourced environment:

```bash
ros2 pkg prefix isaac_manipulator_pose_server
ros2 pkg prefix isaac_manipulator_server_interfaces
ros2 pkg prefix isaac_manipulator_interfaces
ros2 pkg prefix isaac_manipulator_bringup
ros2 pkg prefix isaac_manipulator_servers
```

If any of these fail, the launch files cannot start correctly until your
workspace/dependencies are built and sourced.

If using `object_detection_model:=YOLOV8`, also pre-check:

```bash
ros2 pkg prefix isaac_ros_custom_bringup
ros2 pkg prefix isaac_ros_yolov8
ros2 pkg prefix isaac_ros_dnn_image_encoder
ros2 pkg prefix isaac_ros_tensor_rt
ros2 pkg prefix isaac_ros_image_proc
```

If `isaac_ros_yolov8` / TensorRT encoder dependencies are missing, install the
Quickstart binary packages:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-isaac-ros-yolov8 \
  ros-jazzy-isaac-ros-dnn-image-encoder \
  ros-jazzy-isaac-ros-tensor-rt
```

### Terminal 2: Play rosbag

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play -l --clock "$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_foundationpose/quickstart.bag"
```

### Terminal 3: Trigger scan

```bash
source /opt/ros/jazzy/setup.bash
source "$ISAAC_ROS_WS/install/setup.bash"

ros2 service call /isaac_manipulator_pose_server/scan_bin_objects \
  isaac_manipulator_server_interfaces/srv/ScanBinObjects \
  "{max_objects: 0, expected_count: 1, object_key: '', clear_objects_before_run: true, clear_objects_after_run: true}"
```

Set `object_key` to a configured `pose_server.available_objects` entry for per-scan object selection.

Example selecting `mustard`:

```bash
ros2 service call /isaac_manipulator_pose_server/scan_bin_objects \
  isaac_manipulator_server_interfaces/srv/ScanBinObjects \
  "{max_objects: 0, expected_count: 1, object_key: 'mustard', clear_objects_before_run: true, clear_objects_after_run: true}"
```

### Optional checks

```bash
ros2 param get /object_detection_server input_img_topic_name
ros2 action list | grep -E '/get_objects|/get_object_pose'
ros2 service list | grep -E '/isaac_manipulator_pose_server/scan_bin_objects|/isaac_manipulator_pose_server/get_last_scan'
```
