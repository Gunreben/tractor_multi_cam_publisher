cmake_minimum_required(VERSION 3.16)
project(tractor_multi_cam_publisher)

# 1. Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# 2. Add your executable
add_executable(tractor_multi_cam_publisher
  src/tractor_multi_cam_publisher.cpp
)

# 3. Tell CMake to link against rclcpp, sensor_msgs, etc.
ament_target_dependencies(tractor_multi_cam_publisher
  rclcpp
  sensor_msgs
)

# If you still want to use pkg_check_modules() for GStreamer:
target_include_directories(tractor_multi_cam_publisher PRIVATE
  ${GSTREAMER_INCLUDE_DIRS}
)
target_link_libraries(tractor_multi_cam_publisher
  ${GSTREAMER_LIBRARIES}
)

# 4. Install the executable
install(TARGETS tractor_multi_cam_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Mark this as an ament package
ament_package()
