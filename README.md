# isaac_manipulator_custom_server

Perception-centered extension package set for Isaac Manipulator.

This repository focuses on one specific capability: robustly retrieving
dynamic, multi-object 6D poses from a scene (for example, bin picking setups),
and exposing them through a stable service interface that other systems can
call on demand.

Top-level packages:

- `isaac_manipulator_pose_server`
- `isaac_manipulator_server_interfaces`

## Scope

This repo provides:

- A persistent scan server that can be triggered repeatedly.
- A service API to rescan and return all currently detected object poses.
- Multi-object aggregation logic on top of Isaac Manipulator perception servers.
- Launch files for perception-only bringup and scan server bringup.

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
cd /workspaces/isaac_ros-dev
source /opt/ros/jazzy/setup.bash

colcon build --packages-select \
  isaac_manipulator_server_interfaces \
  isaac_manipulator_pose_server

source install/setup.bash

ros2 launch isaac_manipulator_pose_server perception_scan_server.launch.py \
  camera_type:=ISAAC_SIM \
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
  rtdetr_engine_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/synthetica_detr/sdetr_grasp.plan \
  refine_engine_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose/refine_trt_engine.plan \
  score_engine_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose/score_trt_engine.plan \
  refine_model_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose/refine_model.onnx \
  score_model_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/models/foundationpose/score_model.onnx \
  mesh_file_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/isaac_ros_foundationpose/soup_can/soup_can.obj \
  texture_path:=/workspaces/isaac_ros-dev/isaac_ros_assets/isaac_ros_foundationpose/soup_can/baked_mesh_tex0.png
```

Default runtime config file:

- `src/isaac_manipulator_custom_server/isaac_manipulator_pose_server/params/pose_server.yaml`

### Terminal 2: Play rosbag

```bash
source /opt/ros/jazzy/setup.bash
ros2 bag play -l /workspaces/isaac_ros-dev/isaac_ros_assets/isaac_ros_foundationpose/quickstart.bag
```

### Terminal 3: Trigger scan

```bash
source /opt/ros/jazzy/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash

ros2 service call /custom_pose_bt/scan_bin_objects \
  isaac_manipulator_server_interfaces/srv/ScanBinObjects \
  "{max_objects: 0, expected_count: 1, clear_objects_before_run: true, clear_objects_after_run: true}"
```

### Optional checks

```bash
ros2 param get /object_detection_server input_img_topic_name
ros2 action list | grep -E '/get_objects|/get_object_pose'
ros2 service list | grep -E '/custom_pose_bt/scan_bin_objects|/custom_pose_bt/get_last_scan'
```
