# ADR: `isaac_manipulator_custom_server` Design and Integration

## Status

Accepted.

## Date

2026-02-09

## Context

This repository adds a perception-centered capability on top of Isaac Manipulator:

- triggerable bin rescans
- multi-object 6D pose extraction
- persistent service endpoint (server does not exit after one run)
- normalized result payload for downstream consumers (e.g. task planners, BT frameworks, custom executors)

The goal is to reuse Isaac Manipulator perception/action servers rather than re-implement object detection and pose estimation.

## Decision Summary

1. Use a persistent ROS 2 service server (`multi_object_pose_server`) as the main API surface.
1. Build the scan workflow as an orchestrator over existing Isaac Manipulator actions/services:
   - `/get_objects`
   - `/get_object_pose`
   - `/add_mesh_to_object`
   - `/assign_name_to_object`
   - `/clear_objects` (optional)
1. Keep the package manipulation-agnostic:
   - no direct robot control dependency in the core runtime logic
   - perception-only launch provided for rosbag and offline validation
1. Return complete scan data in one response:
   - object IDs, classes, scores, frame names, poses, and JSON summary
1. Keep runtime configurable via YAML (`params/pose_server.yaml`) and request-time controls for per-scan behavior.

## Package Boundaries

- `isaac_manipulator_pose_server`:
  - runtime server
  - pipeline orchestration
  - launch files
- `isaac_manipulator_server_interfaces`:
  - service contracts (`ScanBinObjects`, `GetLastScan`)

This repo depends on Isaac Manipulator interfaces/servers and is intentionally not a full orchestration stack.

Public services and output topics use `/isaac_manipulator_pose_server/*`.

## Pipeline Design

`BinObjectPosePipeline.run_scan()` executes:

1. Optional `ClearObjects` before run (controlled by request flag).
1. `GetObjects` action call.
1. Filter by configured `target_class_ids` (if set).
1. Optional shared mesh assignment (`AddMeshToObject`) for selected objects.
1. Per-object frame naming (`AssignNameToObject`).
1. Per-object pose request (`GetObjectPose`).
1. Build and publish:
   - `PoseArray` on configured topic (`/isaac_manipulator_pose_server/object_poses` by default)
   - JSON summary on configured topic (`/isaac_manipulator_pose_server/object_pose_summary` by default)
1. Optional `ClearObjects` after run (controlled by request flag).

Failures are fail-fast with explicit error messages including action/service and status.

## Why This Approach

### Reuse of Isaac Manipulator

- Leverages tested action/service behavior already used by Isaac Manipulator.
- Keeps custom code focused on orchestration and API packaging.
- Reduces divergence from upstream and maintenance cost.

### Persistent Server Model

- Better fit for external orchestrators (BTs, task-level planners, APIs) that need repeated rescans.
- Avoids launch/teardown overhead and one-shot behavior.
- Provides `GetLastScan` for low-latency consumers that do not require immediate rescan.

### Service Contract for Aggregated Results

- Downstream clients get a single response containing all object pose records.
- Simplifies integration with systems that do not want to manage multiple action/service calls.

## Launch Strategy

Three launch modes are provided:

1. `scan_server.launch.py`
   - starts only `multi_object_pose_server`
   - use when upstream perception/servers are already running

1. `scan_server_with_pipeline.launch.py`
   - includes Isaac Manipulator `workflows.launch.py` (optional via `launch_pipeline`)
   - starts server + broader manipulator stack

1. `perception_scan_server.launch.py`
   - starts a perception-only stack needed by this repo:
     - RT-DETR pipeline
     - FoundationPose pipeline
     - object detection server
     - foundation pose server
     - object info server
     - `multi_object_pose_server`
   - intended for perception-only validation and rosbag-driven testing
   - avoids requiring UR driver/MoveIt/cuMotion/nvblox when not needed

## Topic and QoS Design Notes

- Input image/camera/depth topics are launch-argument driven.
- `perception_scan_server.launch.py` snapshots user inputs into dedicated launch configs (`scan_*`) to avoid nested include argument collisions.
- QoS is configurable (`input_qos`, `output_qos`) to match live sensors vs rosbag playback scenarios.

## Configuration Model

Runtime configuration expects a `pose_server` YAML root section.
For compatibility, top-level key-value YAML is also accepted.

Key fields:

- filtering: `target_class_ids`
- mesh behavior: `shared_mesh_file_path`
- naming: `object_frame_prefix`
- object limits: `max_objects`
- endpoint names: action/service names (for custom remaps)
- timing: `action_timeout_sec`, `service_timeout_sec`
- publishing: `output_frame_id`, output topics

Request-level overrides:

- `ScanBinObjects.max_objects`
  - `0`: all
  - `< 0`: fallback to configured YAML default
- `ScanBinObjects.clear_objects_before_run`: controls cache clear before scan
- `ScanBinObjects.clear_objects_after_run`: controls cache clear after scan
- `expected_count`
  - if `> 0`, response marks unmet count as unsuccessful (`expected_count_met=false`)

## Intended Usage Pattern

1. Start one of the provided launch modes (usually `perception_scan_server.launch.py` for perception-only use).
1. Trigger scans via `/isaac_manipulator_pose_server/scan_bin_objects`.
1. Consume response payload directly, or consume published `PoseArray`/summary topics.
1. Use `/isaac_manipulator_pose_server/get_last_scan` when a cached result is sufficient.

This design is intended for systems that need dynamic multi-object poses on demand, not only at startup.

## Consequences

### Positive

- Clear separation between perception serving and manipulation execution.
- Reusable from multiple clients without duplicating Isaac Manipulator internals.
- Easier testing with rosbags and perception-only launch.

### Tradeoffs

- Depends on correctness/availability of upstream Isaac Manipulator servers.
- End-to-end behavior inherits upstream detection/pose quality and timeout behavior.

## Future Improvements

1. Add optional deduplication/association post-processing for identical-class objects when detector output includes ambiguous duplicates.
1. Consider exposing server endpoint names for scan/get-last services through parameters to simplify namespace remapping without code changes.
1. Add automated integration test with rosbag playback and expected response schema assertions.
