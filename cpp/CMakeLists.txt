cmake_minimum_required(VERSION 3.8)
project(ros2_yolo_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(detection_logger src/detection_logger.cpp)
ament_target_dependencies(detection_logger rclcpp std_msgs)

install(TARGETS
  detection_logger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
