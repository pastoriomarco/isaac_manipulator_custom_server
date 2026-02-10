# isaac_manipulator_pose_server

Service-first package for multi-object FoundationPose use in bin scenes, with optional same-object-type per-scan selection, outside the full pick-and-place orchestration workflow.

## What it does

The scan pipeline executes this sequence:

1. `GetObjects` (`/get_objects`)
1. Optional per-scan object selection (`object_key`) from `available_objects`
1. Optional class-id filtering (`target_class_ids`)
1. Optional shared mesh assignment for all selected objects (`/add_mesh_to_object`)
1. Per-object TF frame naming (`/assign_name_to_object`)
1. Per-object pose estimation (`/get_object_pose`)
1. Optional cache clear (`/clear_objects`)
1. Publish:
   - `geometry_msgs/PoseArray` on `/isaac_manipulator_pose_server/object_poses`
   - JSON summary on `/isaac_manipulator_pose_server/object_pose_summary`

The package provides a persistent scan server (`multi_object_pose_server`) that stays alive and
exposes services:

- `/isaac_manipulator_pose_server/scan_bin_objects`
- `/isaac_manipulator_pose_server/get_last_scan`

## Required upstream servers/topics

Before running this package, you must have the Isaac Manipulator perception/servers pipeline up, including:

- `/get_objects` action server
- `/get_object_pose` action server
- `/add_mesh_to_object` service
- `/assign_name_to_object` service
- (optional) `/clear_objects` service

These are typically provided by `isaac_manipulator_servers` + perception bringup.

Quick pre-check:

```bash
ros2 pkg prefix isaac_manipulator_pose_server
ros2 pkg prefix isaac_manipulator_server_interfaces
ros2 pkg prefix isaac_manipulator_interfaces
ros2 pkg prefix isaac_manipulator_bringup
ros2 pkg prefix isaac_manipulator_servers
```

## Attribution

This custom package relies heavily on NVIDIA Isaac ROS / Isaac Manipulator interfaces and servers (`isaac_manipulator_interfaces`, `isaac_manipulator_servers`).

## Run

### Recommended: persistent scan server (stays alive)

```bash
ros2 launch isaac_manipulator_pose_server scan_server.launch.py
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

This launch starts a perception-only stack required by this package:

- RT-DETR perception pipeline
- FoundationPose node
- Object detection/foundation pose/object info servers
- `multi_object_pose_server`

It does not launch UR driver, MoveIt, cuMotion, or nvblox.

```bash
ros2 launch isaac_manipulator_pose_server perception_scan_server.launch.py
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
