import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def _validate_detector_setup(
        context,
        *,
        has_yolov8_bringup_package: bool,
        missing_yolov8_runtime_packages):
    detector = context.perform_substitution(LaunchConfiguration('object_detection_model'))
    if detector == 'YOLOV8':
        if not has_yolov8_bringup_package:
            raise RuntimeError(
                'object_detection_model=YOLOV8 requires package "isaac_ros_custom_bringup" '
                '(with launch/yolov8_inference.launch.py) to be available in the sourced workspace.'
            )
        if missing_yolov8_runtime_packages:
            missing = ', '.join(missing_yolov8_runtime_packages)
            raise RuntimeError(
                'object_detection_model=YOLOV8 is missing required runtime package(s): '
                f'{missing}. Build/install them and source the workspace before launching.'
            )
    return []


def generate_launch_description():
    custom_share = get_package_share_directory('isaac_manipulator_pose_server')
    bringup_share = get_package_share_directory('isaac_manipulator_bringup')
    servers_share = get_package_share_directory('isaac_manipulator_servers')
    has_yolov8_bringup_package = True
    try:
        yolo_bringup_share = get_package_share_directory('isaac_ros_custom_bringup')
    except PackageNotFoundError:
        yolo_bringup_share = ''
        has_yolov8_bringup_package = False

    yolov8_runtime_packages = [
        'isaac_ros_yolov8',
        'isaac_ros_dnn_image_encoder',
        'isaac_ros_tensor_rt',
        'isaac_ros_image_proc',
    ]
    missing_yolov8_runtime_packages = []
    for package_name in yolov8_runtime_packages:
        try:
            get_package_share_directory(package_name)
        except PackageNotFoundError:
            missing_yolov8_runtime_packages.append(package_name)

    default_config = os.path.join(custom_share, 'params', 'pose_server.yaml')

    launch_args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Path to pose server config YAML.'
        ),
        DeclareLaunchArgument('camera_type', default_value='REALSENSE'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to use simulation time for included pipelines.',
        ),
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
        DeclareLaunchArgument(
            'object_detection_model',
            default_value='RT_DETR',
            choices=['RT_DETR', 'YOLOV8'],
            description='Object detector frontend used for GetObjects (RT_DETR or YOLOV8).',
        ),
        DeclareLaunchArgument('rtdetr_engine_file_path', default_value='/tmp/rtdetr.plan'),
        DeclareLaunchArgument('rt_detr_confidence_threshold', default_value='0.7'),
        DeclareLaunchArgument('object_class_id', default_value='22'),
        DeclareLaunchArgument('yolov8_model_file_path', default_value='/tmp/yolov8.onnx'),
        DeclareLaunchArgument('yolov8_engine_file_path', default_value='/tmp/yolov8.plan'),
        DeclareLaunchArgument('yolov8_input_tensor_names', default_value='["input_tensor"]'),
        DeclareLaunchArgument('yolov8_input_binding_names', default_value='["images"]'),
        DeclareLaunchArgument('yolov8_output_tensor_names', default_value='["output_tensor"]'),
        DeclareLaunchArgument('yolov8_output_binding_names', default_value='["output0"]'),
        DeclareLaunchArgument('yolov8_confidence_threshold', default_value='0.25'),
        DeclareLaunchArgument('yolov8_nms_threshold', default_value='0.45'),
        DeclareLaunchArgument('yolov8_num_classes', default_value='80'),
        DeclareLaunchArgument('yolov8_network_image_width', default_value='640'),
        DeclareLaunchArgument('yolov8_network_image_height', default_value='640'),
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
        DeclareLaunchArgument(
            'discard_old_messages',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument('discard_msg_older_than_ms', default_value='1000'),
        DeclareLaunchArgument(
            'enable_dnn_depth_in_realsense',
            default_value='false',
            choices=['true', 'false'],
        ),
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
        SetLaunchConfiguration(
            'scan_discard_old_messages',
            PythonExpression(
                [
                    "'True' if '",
                    LaunchConfiguration('discard_old_messages'),
                    "'.lower() == 'true' else 'False'",
                ]
            ),
        ),
        SetLaunchConfiguration(
            'scan_enable_dnn_depth_in_realsense',
            PythonExpression(
                [
                    "'True' if '",
                    LaunchConfiguration('enable_dnn_depth_in_realsense'),
                    "'.lower() == 'true' else 'False'",
                ]
            ),
        ),
    ]
    detector_is_rtdetr = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration('object_detection_model'), "' == 'RT_DETR'"]
        )
    )
    detector_is_yolov8 = IfCondition(
        PythonExpression(
            ["'", LaunchConfiguration('object_detection_model'), "' == 'YOLOV8'"]
        )
    )
    validate_detector_setup = OpaqueFunction(
        function=_validate_detector_setup,
        kwargs={
            'has_yolov8_bringup_package': has_yolov8_bringup_package,
            'missing_yolov8_runtime_packages': missing_yolov8_runtime_packages,
        },
    )

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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'foundationpose_server_input_camera_info_topic': '/foundation_pose_server/camera_info',
        }.items(),
        condition=detector_is_rtdetr,
    )

    # YOLOv8 detector pipeline from local custom bringup package.
    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_bringup_share, 'launch', 'yolov8_inference.launch.py')
        ),
        launch_arguments={
            'model_file_path': LaunchConfiguration('yolov8_model_file_path'),
            'engine_file_path': LaunchConfiguration('yolov8_engine_file_path'),
            'input_tensor_names': LaunchConfiguration('yolov8_input_tensor_names'),
            'input_binding_names': LaunchConfiguration('yolov8_input_binding_names'),
            'output_tensor_names': LaunchConfiguration('yolov8_output_tensor_names'),
            'output_binding_names': LaunchConfiguration('yolov8_output_binding_names'),
            'confidence_threshold': LaunchConfiguration('yolov8_confidence_threshold'),
            'nms_threshold': LaunchConfiguration('yolov8_nms_threshold'),
            'num_classes': LaunchConfiguration('yolov8_num_classes'),
            'network_image_width': LaunchConfiguration('yolov8_network_image_width'),
            'network_image_height': LaunchConfiguration('yolov8_network_image_height'),
            'image_input_topic': LaunchConfiguration('scan_rgb_image_topic'),
            'camera_info_input_topic': LaunchConfiguration('scan_rgb_camera_info_topic'),
        }.items(),
        condition=detector_is_yolov8,
    )

    # For YOLOv8 mode, build FoundationPose masks from the per-request bbox emitted by
    # foundation_pose_server (/foundation_pose_server/bbox), analogous to RT-DETR ROI mode.
    yolov8_detection2d_to_mask_node = ComposableNode(
        name='yolov8_detection2_d_to_mask',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::Detection2DToMask',
        parameters=[{
            'mask_width': LaunchConfiguration('scan_rgb_image_width'),
            'mask_height': LaunchConfiguration('scan_rgb_image_height'),
            'input_qos': LaunchConfiguration('scan_output_qos'),
            'output_qos': LaunchConfiguration('scan_output_qos'),
        }],
        remappings=[
            ('detection2_d', '/foundation_pose_server/bbox'),
            ('segmentation', 'yolov8_segmentation_rgb'),
        ],
    )
    yolov8_resize_mask_node = ComposableNode(
        name='yolov8_resize_mask_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'input_width': LaunchConfiguration('scan_rgb_image_width'),
            'input_height': LaunchConfiguration('scan_rgb_image_height'),
            'output_width': LaunchConfiguration('scan_depth_image_width'),
            'output_height': LaunchConfiguration('scan_depth_image_height'),
            'keep_aspect_ratio': False,
            'disable_padding': False,
            'encoding_desired': 'mono8',
            'input_qos': LaunchConfiguration('scan_output_qos'),
            'output_qos': LaunchConfiguration('scan_output_qos'),
            'use_latest_camera_info': True,
            'drop_old_messages': False,
        }],
        remappings=[
            ('image', 'yolov8_segmentation_rgb'),
            ('camera_info', '/foundation_pose_server/camera_info'),
            ('resize/image', 'segmentation'),
            ('resize/camera_info', 'camera_info_segmentation'),
        ],
    )
    yolov8_mask_nodes = LoadComposableNodes(
        target_container='manipulator_container',
        composable_node_descriptions=[
            yolov8_detection2d_to_mask_node,
            yolov8_resize_mask_node,
        ],
        condition=detector_is_yolov8,
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
            'discard_old_messages': LaunchConfiguration('scan_discard_old_messages'),
            'discard_msg_older_than_ms': LaunchConfiguration('discard_msg_older_than_ms'),
            'enable_dnn_depth_in_realsense':
                LaunchConfiguration('scan_enable_dnn_depth_in_realsense'),
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
            # Keep backend value RT_DETR for DetectObjects-action-compatible detectors
            # (RT-DETR and YOLOv8 both route through object_detection_server).
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
        validate_detector_setup,
        manipulator_container,
        rtdetr_launch,
        yolov8_launch,
        yolov8_mask_nodes,
        foundationpose_launch,
        object_detection_server_launch,
        foundation_pose_server_launch,
        object_info_server_launch,
        scan_server,
    ])
