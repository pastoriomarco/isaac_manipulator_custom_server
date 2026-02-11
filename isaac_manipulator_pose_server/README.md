# isaac_manipulator_pose_server

Service-first package for multi-object FoundationPose use in bin scenes, with optional same-object-type per-scan selection, outside the full pick-and-place orchestration workflow.

## What it does

This package supports two scan modes:

1. Legacy mode (`multi_object_pose_server`) through `object_info_server`:
   1. `GetObjects` (`/get_objects`)
   1. Optional per-scan object selection (`object_key`) from `available_objects`
   1. Optional class-id filtering (`target_class_ids`)
   1. Optional shared mesh assignment for all selected objects (`/add_mesh_to_object`)
   1. Per-object TF frame naming (`/assign_name_to_object`)
   1. Per-object pose estimation (`/get_object_pose`)
   1. Optional cache clear (`/clear_objects`)
2. Direct mode (`multi_object_pose_server_direct`) without `object_info_server`:
   1. `DetectObjects` (`/detect_objects`)
   1. Optional class-id filtering (`target_class_ids`) + class-aware NMS (`nms_iou_threshold`)
   1. Per-detection FoundationPose (`/estimate_pose_foundation_pose`) using `shared_mesh_file_path`
   1. Optional best-effort cache clear (`/clear_objects`)

Both modes publish:

- `geometry_msgs/PoseArray` on `/isaac_manipulator_pose_server/object_poses`
- JSON summary on `/isaac_manipulator_pose_server/object_pose_summary`

Both servers expose services:

- `/isaac_manipulator_pose_server/scan_bin_objects`
- `/isaac_manipulator_pose_server/get_last_scan`

## Required upstream servers/topics

For legacy mode (`multi_object_pose_server`), you need:

- `/get_objects` action server
- `/get_object_pose` action server
- `/add_mesh_to_object` service
- `/assign_name_to_object` service
- (optional) `/clear_objects` service

For direct mode (`multi_object_pose_server_direct`), you need:

- `/detect_objects` action server
- `/estimate_pose_foundation_pose` action server
- (optional) `/clear_objects` service

These are provided by `isaac_manipulator_servers` plus detector/FoundationPose bringup.

Quick pre-check:

```bash
ros2 pkg prefix isaac_manipulator_pose_server
ros2 pkg prefix isaac_manipulator_server_interfaces
ros2 pkg prefix isaac_manipulator_interfaces
ros2 pkg prefix isaac_manipulator_bringup
ros2 pkg prefix isaac_manipulator_servers
```

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

## Attribution

This custom package relies heavily on NVIDIA Isaac ROS / Isaac Manipulator interfaces and servers (`isaac_manipulator_interfaces`, `isaac_manipulator_servers`).

## Run

### Recommended: persistent scan server (stays alive)

```bash
ros2 launch isaac_manipulator_pose_server scan_server.launch.py
```

Direct-mode server only:

```bash
ros2 launch isaac_manipulator_pose_server scan_server_direct.launch.py
```

Trigger a scan:

```bash
ros2 service call /isaac_manipulator_pose_server/scan_bin_objects \
  isaac_manipulator_server_interfaces/srv/ScanBinObjects \
  "{max_objects: 0, expected_count: 0, object_key: '', clear_objects_before_run: false, clear_objects_after_run: true}"
```

Read last result (without rescanning):

```bash
ros2 service call /isaac_manipulator_pose_server/get_last_scan \
  isaac_manipulator_server_interfaces/srv/GetLastScan \
  "{}"
```

### Easiest bringup: pipeline + scan server in one launch

```bash
ros2 launch isaac_manipulator_pose_server scan_server_with_pipeline.launch.py
```

Override Isaac Manipulator workflow config:

```bash
ros2 launch isaac_manipulator_pose_server scan_server_with_pipeline.launch.py \
  manipulator_workflow_config:=/path/to/isaac_manipulator_workflow.yaml
```

### Recommended for rosbag/perception-only usage

Legacy perception stack + legacy scan server:

This launch starts a perception-only stack required by this package:

- Selectable object detector (`RT_DETR` or `YOLOV8`)
- FoundationPose node
- Object detection/foundation pose/object info servers
- `multi_object_pose_server`

It does not launch UR driver, MoveIt, cuMotion, or nvblox.

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
  rt_detr_confidence_threshold:=0.3 \
  rtdetr_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan" \
  refine_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan" \
  score_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/score_trt_engine.plan" \
  refine_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/refine_model.onnx" \
  score_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/score_model.onnx" \
  mesh_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_foundationpose/soup_can/soup_can.obj" \
  texture_path:="$ISAAC_ROS_WS/isaac_ros_assets/isaac_ros_foundationpose/soup_can/baked_mesh_tex0.png"
```

Direct perception stack + direct scan server (recommended when you want to bypass
`object_info_server` and query detector + FoundationPose actions directly):

```bash
ros2 launch isaac_manipulator_pose_server perception_scan_server_direct.launch.py \
  camera_type:=ISAAC_SIM \
  use_sim_time:=false \
  foundationpose_depth_topic:=/foundation_pose_server/depth \
  rgb_image_topic:=/image_rect \
  rgb_camera_info_topic:=/camera_info \
  depth_image_topic:=/depth \
  depth_camera_info_topic:=/camera_info \
  rgb_image_width:=1280 rgb_image_height:=720 \
  depth_image_width:=1280 depth_image_height:=720 \
  input_qos:=DEFAULT output_qos:=DEFAULT \
  input_fps:=8 dropped_fps:=8 \
  rt_detr_confidence_threshold:=0.3 \
  rtdetr_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/rt_detr_custom/model_isaac.plan" \
  refine_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan" \
  score_engine_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/score_trt_engine.plan" \
  refine_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/refine_model.onnx" \
  score_model_file_path:="$ISAAC_ROS_WS/isaac_ros_assets/models/foundationpose/score_model.onnx" \
  mesh_file_path:="$ISAAC_ROS_WS/src/isaac_sim_custom_examples/trocar_short.obj" \
  texture_path:="$ISAAC_ROS_WS/src/isaac_sim_custom_examples/grey.png"
```

Detector selection is launch-time configurable:

- `object_detection_model:=RT_DETR` (default)
- `object_detection_model:=YOLOV8`

Example with YOLOv8 detector frontend:

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

For rosbags that already publish metric depth (`32FC1`), run FoundationPose in
non-convert-metric mode:

```bash
ros2 launch isaac_manipulator_pose_server perception_scan_server.launch.py \
  camera_type:=ISAAC_SIM \
  use_sim_time:=true \
  foundationpose_depth_topic:=/foundation_pose_server/depth
```

When using `use_sim_time:=true` with rosbag playback, run `ros2 bag play --clock ...`.

## Config

Default config file:

- `params/pose_server.yaml`

Important fields:

- `target_class_ids`: e.g. `['3']` for soup can only (empty means all)
- `shared_mesh_file_path`: one `.obj` mesh used for all selected IDs
- `available_objects`: object-key catalog used by per-scan `object_key` selection
- `object_frame_prefix`: outputs names like `bin_object_0`, `bin_object_1`, ...
- `max_objects`: limit how many detections to process (`0` = all)
- `detect_objects_action_name`: direct-mode detector action endpoint
- `estimate_pose_action_name`: direct-mode FoundationPose action endpoint
- `action_retry_count`: retries for action waits/calls in direct mode
- `estimate_pose_retry_count`: retries for each FoundationPose action in direct mode
- `additional_pose_timeout_sec`: shorter timeout used for pose attempts after the first pose attempt in a multi-object scan
- `retry_backoff_sec`: delay between retries in direct mode
- `nms_iou_threshold`: IoU threshold for class-aware suppression in direct mode
- `enable_bbox_memory`: keep short-term memory of successful boxes in direct mode
- `bbox_memory_center_distance_px`: minimum center distance for a new box vs remembered boxes
- `bbox_memory_ttl_sec`: lifetime of remembered boxes
- `max_detection_rounds_per_scan`: max detect rounds when collecting `max_objects > 1`
- `one_pose_per_detection_round`: if `false`, try multiple candidates from a detect round before re-detecting (faster)

For direct mode, keep `action_timeout_sec` high enough for first-run TensorRT warmup
(for example `45.0` seconds).

For multi-object scans across detected classes, leave `object_key` empty and keep
`target_class_ids` empty.

For same-object-type scans without `object_key`, keep `target_class_ids`
constrained to a single class id.

Example object catalog:

```yaml
pose_server:
  available_objects:
    soup_can:
      class_id: '3'
      mesh_file_path: '/absolute/path/to/soup_can.obj'
    tuna_can:
      class_id: '8'
      mesh_file_path: '/absolute/path/to/tuna_can.obj'
```

## Service semantics

`ScanBinObjects` request:

- `max_objects`: `0` means all, `< 0` means use YAML-configured default.
- `expected_count`: if `> 0`, response sets `expected_count_met` and returns `success=false` when unmet.
- `object_key`: optional key from `pose_server.available_objects`; overrides class filter and mesh for this scan.
- `clear_objects_before_run`: clear object cache before scanning.
- `clear_objects_after_run`: clear object cache after scanning.

Example selecting an object profile per scan:

```bash
ros2 service call /isaac_manipulator_pose_server/scan_bin_objects \
  isaac_manipulator_server_interfaces/srv/ScanBinObjects \
  "{max_objects: 0, expected_count: 0, object_key: 'soup_can', clear_objects_before_run: true, clear_objects_after_run: true}"
```

Example targeting `mustard` (when `available_objects.mustard` is configured):

```bash
ros2 service call /isaac_manipulator_pose_server/scan_bin_objects \
  isaac_manipulator_server_interfaces/srv/ScanBinObjects \
  "{max_objects: 0, expected_count: 1, object_key: 'mustard', clear_objects_before_run: true, clear_objects_after_run: true}"
```
