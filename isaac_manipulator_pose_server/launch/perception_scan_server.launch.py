import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    custom_share = get_package_share_directory('isaac_manipulator_pose_server')
    bringup_share = get_package_share_directory('isaac_manipulator_bringup')
    servers_share = get_package_share_directory('isaac_manipulator_servers')

    default_config = os.path.join(custom_share, 'params', 'pose_server.yaml')

    launch_args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to pose server config YAML.'
        ),
        DeclareLaunchArgument('camera_type', default_value='REALSENSE'),
        DeclareLaunchArgument(
            'foundationpose_depth_topic',
            default_value='/foundation_pose_server/depth_metric',
            description=(
                'Depth topic consumed by FoundationPose. Use /foundation_pose_server/depth_metric '
                'for REALSENSE MONO16->metric conversion, or /foundation_pose_server/depth for '
                'already-metric depth (e.g., 32FC1 rosbags).'
            ),
        ),
        DeclareLaunchArgument('rgb_image_topic', default_value='/camera_1/color/image_raw'),
        DeclareLaunchArgument('rgb_camera_info_topic', default_value='/camera_1/color/camera_info'),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera_1/aligned_depth_to_color/image_raw'
        ),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera_1/aligned_depth_to_color/camera_info'
        ),
        DeclareLaunchArgument('rgb_image_width', default_value='1280'),
        DeclareLaunchArgument('rgb_image_height', default_value='720'),
        DeclareLaunchArgument('depth_image_width', default_value='1280'),
        DeclareLaunchArgument('depth_image_height', default_value='720'),
        DeclareLaunchArgument('rtdetr_engine_file_path', default_value='/tmp/rtdetr.plan'),
        DeclareLaunchArgument('rt_detr_confidence_threshold', default_value='0.7'),
        DeclareLaunchArgument('object_class_id', default_value='22'),
        DeclareLaunchArgument('input_fps', default_value='30'),
        DeclareLaunchArgument('dropped_fps', default_value='28'),
        DeclareLaunchArgument('input_qos', default_value='SENSOR_DATA'),
        DeclareLaunchArgument('output_qos', default_value='DEFAULT'),
        DeclareLaunchArgument('mesh_file_path', default_value='/tmp/textured_simple.obj'),
        DeclareLaunchArgument('texture_path', default_value='/tmp/texture_map.png'),
        DeclareLaunchArgument('refine_model_file_path', default_value='/tmp/refine_model.onnx'),
        DeclareLaunchArgument('refine_engine_file_path', default_value='/tmp/refine_trt_engine.plan'),
        DeclareLaunchArgument('score_model_file_path', default_value='/tmp/score_model.onnx'),
        DeclareLaunchArgument('score_engine_file_path', default_value='/tmp/score_trt_engine.plan'),
        DeclareLaunchArgument('refine_iterations', default_value='3'),
        DeclareLaunchArgument('symmetry_axes', default_value='["x_180", "y_180", "z_180"]'),
        DeclareLaunchArgument('discard_old_messages', default_value='true'),
        DeclareLaunchArgument('discard_msg_older_than_ms', default_value='1000'),
        DeclareLaunchArgument('enable_dnn_depth_in_realsense', default_value='false'),
        DeclareLaunchArgument('tf_frame_name', default_value='detected_object1'),
    ]

    # Snapshot user-facing inputs into dedicated keys so nested includes that reuse common
    # argument names (e.g. rgb_image_topic) cannot override the values used by this launch.
    capture_inputs = [
        SetLaunchConfiguration('scan_camera_type', LaunchConfiguration('camera_type')),
        SetLaunchConfiguration('scan_rgb_image_topic', LaunchConfiguration('rgb_image_topic')),
        SetLaunchConfiguration(
            'scan_rgb_camera_info_topic', LaunchConfiguration('rgb_camera_info_topic')),
        SetLaunchConfiguration('scan_depth_image_topic', LaunchConfiguration('depth_image_topic')),
        SetLaunchConfiguration(
            'scan_depth_camera_info_topic', LaunchConfiguration('depth_camera_info_topic')),
        SetLaunchConfiguration('scan_rgb_image_width', LaunchConfiguration('rgb_image_width')),
        SetLaunchConfiguration('scan_rgb_image_height', LaunchConfiguration('rgb_image_height')),
        SetLaunchConfiguration('scan_depth_image_width', LaunchConfiguration('depth_image_width')),
        SetLaunchConfiguration(
            'scan_depth_image_height', LaunchConfiguration('depth_image_height')),
        SetLaunchConfiguration(
            'scan_foundationpose_depth_topic', LaunchConfiguration('foundationpose_depth_topic')),
        SetLaunchConfiguration('scan_input_qos', LaunchConfiguration('input_qos')),
        SetLaunchConfiguration('scan_output_qos', LaunchConfiguration('output_qos')),
    ]

    # Standalone component container matching MANIPULATOR_CONTAINER_NAME.
    manipulator_container = Node(
        name='manipulator_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
    )

    # RT-DETR perception pipeline + bbox->mask generation for FoundationPose (ROI mode).
    rtdetr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'include', 'rtdetr.launch.py')
        ),
        launch_arguments={
            'image_width': LaunchConfiguration('scan_rgb_image_width'),
            'image_height': LaunchConfiguration('scan_rgb_image_height'),
            'depth_image_width': LaunchConfiguration('scan_depth_image_width'),
            'depth_image_height': LaunchConfiguration('scan_depth_image_height'),
            'image_input_topic': LaunchConfiguration('scan_rgb_image_topic'),
            'depth_topic_name': LaunchConfiguration('scan_depth_image_topic'),
            'camera_info_input_topic': LaunchConfiguration('scan_rgb_camera_info_topic'),
            'rtdetr_engine_file_path': LaunchConfiguration('rtdetr_engine_file_path'),
            'rt_detr_confidence_threshold': LaunchConfiguration('rt_detr_confidence_threshold'),
            'detections_2d_array_output_topic': 'detections_output',
            'rtdetr_is_object_following': 'False',
            'object_class_id': LaunchConfiguration('object_class_id'),
            'detection2_d_topic': '/foundation_pose_server/bbox',
            'input_fps': LaunchConfiguration('input_fps'),
            'dropped_fps': LaunchConfiguration('dropped_fps'),
            'input_qos': LaunchConfiguration('scan_input_qos'),
            'output_qos': LaunchConfiguration('scan_output_qos'),
            'foundationpose_server_input_camera_info_topic': '/foundation_pose_server/camera_info',
        }.items(),
    )

    # FoundationPose DNN node pipeline.
    foundationpose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'include', 'foundationpose.launch.py')
        ),
        launch_arguments={
            'camera_type': LaunchConfiguration('scan_camera_type'),
            'rgb_image_topic': '/foundation_pose_server/image',
            'rgb_image_width': LaunchConfiguration('scan_rgb_image_width'),
            'rgb_image_height': LaunchConfiguration('scan_rgb_image_height'),
            'depth_image_width': LaunchConfiguration('scan_depth_image_width'),
            'depth_image_height': LaunchConfiguration('scan_depth_image_height'),
            'realsense_depth_image_topic': '/foundation_pose_server/depth',
            'foundation_pose_server_depth_topic_name': LaunchConfiguration(
                'scan_foundationpose_depth_topic'),
            'realsense_depth_camera_info_topic': LaunchConfiguration(
                'scan_depth_camera_info_topic'),
            'segmentation_mask_camera_info_topic': '/foundation_pose_server/camera_info',
            'segmentation_mask_topic': 'segmentation',
            'output_pose_estimate_topic': '/pose_estimation/output',
            'mesh_file_path': LaunchConfiguration('mesh_file_path'),
            'texture_path': LaunchConfiguration('texture_path'),
            'refine_model_file_path': LaunchConfiguration('refine_model_file_path'),
            'refine_engine_file_path': LaunchConfiguration('refine_engine_file_path'),
            'score_model_file_path': LaunchConfiguration('score_model_file_path'),
            'score_engine_file_path': LaunchConfiguration('score_engine_file_path'),
            'refine_iterations': LaunchConfiguration('refine_iterations'),
            'symmetry_axes': LaunchConfiguration('symmetry_axes'),
            'tf_frame_name': LaunchConfiguration('tf_frame_name'),
            'foundationpose_sensor_qos_config': LaunchConfiguration('scan_input_qos'),
            'discard_old_messages': LaunchConfiguration('discard_old_messages'),
            'discard_msg_older_than_ms': LaunchConfiguration('discard_msg_older_than_ms'),
            'enable_dnn_depth_in_realsense': LaunchConfiguration('enable_dnn_depth_in_realsense'),
        }.items(),
    )

    # Detection server maps detector outputs into DetectObjects action.
    object_detection_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(servers_share, 'launch', 'object_detection_server.launch.py')
        ),
        launch_arguments={
            'obj_input_img_topic_name': LaunchConfiguration('scan_rgb_image_topic'),
            'obj_output_img_topic_name': '/object_detection_server/image_rect',
            'obj_input_camera_info_topic_name': LaunchConfiguration('scan_rgb_camera_info_topic'),
            'obj_output_camera_info_topic_name': '/object_detection_server/camera_info',
            'obj_input_detections_topic_name': 'detections_output',
            'obj_output_detections_topic_name': '/object_detection_server/detections_output',
            'obj_input_qos': LaunchConfiguration('scan_input_qos'),
            'obj_result_and_output_qos': LaunchConfiguration('scan_output_qos'),
        }.items(),
    )

    # FoundationPose server wraps FP action endpoint used by ObjectInfoServer.
    foundation_pose_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(servers_share, 'launch', 'foundation_pose_server.launch.py')
        ),
        launch_arguments={
            'fp_in_img_topic_name': LaunchConfiguration('scan_rgb_image_topic'),
            'fp_out_img_topic_name': '/foundation_pose_server/image',
            'fp_in_camera_info_topic_name': LaunchConfiguration('scan_rgb_camera_info_topic'),
            'fp_out_camera_info_topic_name': '/foundation_pose_server/camera_info',
            'fp_in_depth_topic_name': LaunchConfiguration('scan_depth_image_topic'),
            'fp_out_depth_topic_name': '/foundation_pose_server/depth',
            'fp_out_bbox_topic_name': '/foundation_pose_server/bbox',
            'fp_in_pose_estimate_topic_name': '/pose_estimation/output',
            'fp_out_pose_estimate_topic_name': '/foundation_pose_server/pose_estimation/output',
            'fp_out_segmented_mask_topic_name': '/foundation_pose_server/segmented_mask',
            'fp_input_qos': LaunchConfiguration('scan_input_qos'),
            'fp_result_and_output_qos': LaunchConfiguration('scan_output_qos'),
        }.items(),
    )

    # ObjectInfoServer exposes get_objects/get_object_pose/add_mesh/assign_name/clear_objects.
    object_info_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(servers_share, 'launch', 'object_info_server.launch.py')
        ),
        launch_arguments={
            'standalone': 'false',
            'object_detection_backend': 'RT_DETR',
            'pose_estimation_backend': 'FOUNDATION_POSE',
            'segmentation_backend': 'NONE',
        }.items(),
    )

    scan_server = Node(
        package='isaac_manipulator_pose_server',
        executable='multi_object_pose_server',
        name='multi_object_pose_server',
        output='screen',
        arguments=['--config-file', LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        *launch_args,
        *capture_inputs,
        manipulator_container,
        rtdetr_launch,
        foundationpose_launch,
        object_detection_server_launch,
        foundation_pose_server_launch,
        object_info_server_launch,
        scan_server,
    ])
