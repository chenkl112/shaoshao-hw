cmake_minimum_required(VERSION 3.16)
project(shaoshao_hw_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/NavCommand.msg" "msg/ChassisFeedback.msg"
  "msg/SpinCommand.msg" "msg/PredictedObstacle.msg" "msg/PredictedObstacles.msg" "srv/RegisterCloud.srv"
  "msg/LocalizationStatus.msg" "msg/NavigationSession.msg" "msg/PlannedTrajectory.msg" "msg/NavcarFeedback.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
