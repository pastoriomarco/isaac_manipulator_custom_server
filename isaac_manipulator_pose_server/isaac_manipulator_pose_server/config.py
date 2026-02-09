from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import yaml


@dataclass(frozen=True)
class WorkflowConfig:
    target_class_ids: List[str]
    shared_mesh_file_path: str
    object_frame_prefix: str
    max_objects: int
    get_objects_action_name: str
    get_object_pose_action_name: str
    add_mesh_service_name: str
    assign_name_service_name: str
    clear_objects_service_name: str
    action_timeout_sec: float
    service_timeout_sec: float
    output_frame_id: str
    output_pose_array_topic: str
    output_summary_topic: str


_DEFAULT_CONFIG: Dict[str, Any] = {
    'target_class_ids': [],
    'shared_mesh_file_path': '',
    'object_frame_prefix': 'bin_object',
    'max_objects': 0,
    'get_objects_action_name': '/get_objects',
    'get_object_pose_action_name': '/get_object_pose',
    'add_mesh_service_name': '/add_mesh_to_object',
    'assign_name_service_name': '/assign_name_to_object',
    'clear_objects_service_name': '/clear_objects',
    'action_timeout_sec': 20.0,
    'service_timeout_sec': 10.0,
    'output_frame_id': 'base_link',
    'output_pose_array_topic': '/isaac_manipulator_pose_server/object_poses',
    'output_summary_topic': '/isaac_manipulator_pose_server/object_pose_summary',
}


def load_config(config_file: str) -> WorkflowConfig:
    config_path = Path(config_file)
    if not config_path.exists():
        raise FileNotFoundError(f'Configuration file not found: {config_path}')

    with config_path.open('r', encoding='utf-8') as file:
        loaded_config = yaml.safe_load(file) or {}

    if not isinstance(loaded_config, dict):
        raise ValueError(f'Invalid config format in {config_path}. Expected a dictionary.')

    if 'pose_server' in loaded_config:
        loaded_config = loaded_config['pose_server'] or {}

    if not isinstance(loaded_config, dict):
        raise ValueError(
            f'Invalid "pose_server" section format in {config_path}. '
            'Expected a dictionary.'
        )

    merged_config = dict(_DEFAULT_CONFIG)
    merged_config.update(loaded_config)

    target_class_ids = [str(class_id) for class_id in merged_config['target_class_ids']]

    return WorkflowConfig(
        target_class_ids=target_class_ids,
        shared_mesh_file_path=str(merged_config['shared_mesh_file_path']),
        object_frame_prefix=str(merged_config['object_frame_prefix']),
        max_objects=int(merged_config['max_objects']),
        get_objects_action_name=str(merged_config['get_objects_action_name']),
        get_object_pose_action_name=str(merged_config['get_object_pose_action_name']),
        add_mesh_service_name=str(merged_config['add_mesh_service_name']),
        assign_name_service_name=str(merged_config['assign_name_service_name']),
        clear_objects_service_name=str(merged_config['clear_objects_service_name']),
        action_timeout_sec=float(merged_config['action_timeout_sec']),
        service_timeout_sec=float(merged_config['service_timeout_sec']),
        output_frame_id=str(merged_config['output_frame_id']),
        output_pose_array_topic=str(merged_config['output_pose_array_topic']),
        output_summary_topic=str(merged_config['output_summary_topic']),
    )
