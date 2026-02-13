cmake_minimum_required(VERSION 3.8)
project(isaac_manipulator_server_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PipelineCommand.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/PipelineStatus.msg"
  "srv/ScanBinObjects.srv"
  "srv/GetLastScan.srv"
  "srv/MarkObjectUsed.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
    vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
