cmake_minimum_required(VERSION 3.16)
project(tractor_multi_cam_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

#
# ---- C++ Executable ----
#
add_executable(tractor_multi_cam_publisher
  src/tractor_multi_cam_publisher.cpp
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

install(
  TARGETS tractor_multi_cam_publisher
  DESTINATION lib/${PROJECT_NAME}
)

#
# ---- Python Node Installation ----
#
# Make sure the Python script has a proper shebang (e.g. #!/usr/bin/env python3)
# and is marked as executable (chmod +x multi_camera_info_publisher.py).
#
install(
  PROGRAMS
    scripts/multi_camera_info_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
