cmake_minimum_required(VERSION 3.16)
project(tractor_multi_cam_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(tractor_multi_cam_publisher
  src/tractor_multi_cam_publisher.cpp
)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher
  rclcpp
  sensor_msgs
  camera_calibration_parsers
)

ament_target_dependencies(tractor_multi_cam_publisher
  rclcpp
  sensor_msgs
)

target_include_directories(tractor_multi_cam_publisher PRIVATE
  ${GSTREAMER_INCLUDE_DIRS}
)
target_link_libraries(tractor_multi_cam_publisher
  ${GSTREAMER_LIBRARIES}
)

install(TARGETS
  tractor_multi_cam_publisher
  camera_info_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the calibration directory containing YAML files
install(DIRECTORY calibration/
  DESTINATION share/${PROJECT_NAME}/calibration
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()
