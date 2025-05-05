// src/camera_info_publisher.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_calibration_parsers/parse.hpp> // For YAML parsing
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>                         // For directory iteration
#include <string>
#include <vector>
#include <map>
#include <fstream> // Include fstream for file checks

namespace fs = std::filesystem;

class CameraInfoPublisher : public rclcpp::Node
{
public:
  CameraInfoPublisher()
  : Node("camera_info_publisher")
  {
    // Get the real path to <pkg>/share/tractor_multi_cam_publisher/calibration
    std::string default_calib = ament_index_cpp::get_package_share_directory("tractor_multi_cam_publisher")
                              + "/calibration";

    std::string calibration_path = this->declare_parameter<std::string>(
        "calibration_path", default_calib);


    // ----- QoS -----
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .best_effort()
                   .durability_volatile();

    // ----- Load calibrations and create publishers -----
    if (!fs::is_directory(calibration_path)) {
      RCLCPP_ERROR(this->get_logger(), "Calibration path '%s' is not a directory or does not exist.", calibration_path.c_str());
      // Optionally throw an exception or handle the error appropriately
      return; // Exit constructor if path is invalid
    }

    for (const auto & entry : fs::directory_iterator(calibration_path)) {
      if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
        std::string file_path = entry.path().string();
        std::string file_stem = entry.path().stem().string(); // e.g., camera_rear_right.intrinsics
        std::string internal_camera_name; // Name read from inside the YAML
        sensor_msgs::msg::CameraInfo camera_info;

        // Extract camera identifier from filename (e.g., rear_right from camera_rear_right.intrinsics)
        std::string extracted_name = file_stem;
        const std::string prefix = "camera_";
        const std::string suffix = ".intrinsics";

        if (extracted_name.rfind(prefix, 0) == 0) { // Check if starts with prefix
            extracted_name.erase(0, prefix.length());
        }
        size_t suffix_pos = extracted_name.rfind(suffix);
        if (suffix_pos != std::string::npos && suffix_pos == extracted_name.length() - suffix.length()) { // Check if ends with suffix
            extracted_name.erase(suffix_pos);
        }

        if (extracted_name.empty() || extracted_name == file_stem) {
            RCLCPP_WARN(this->get_logger(), "Could not extract meaningful name from filename stem '%s'. Skipping file %s",
                        file_stem.c_str(), file_path.c_str());
            continue; // Skip this file if name extraction failed
        }

        RCLCPP_INFO(this->get_logger(), "Attempting to load calibration from: %s for camera ID: %s",
                     file_path.c_str(), extracted_name.c_str());

        // Read calibration data, using internal_camera_name just to fulfill the API
        if (camera_calibration_parsers::readCalibration(file_path, internal_camera_name, camera_info)) {
          RCLCPP_INFO(this->get_logger(), "Successfully loaded calibration data from %s (Internal name: %s)",
                       file_path.c_str(), internal_camera_name.c_str());

          // Use the extracted name for topic and frame_id
          camera_info.header.frame_id = extracted_name;

          // Create publisher using the extracted name for the topic
          auto topic = std::string("/camera/") + extracted_name + "/camera_info";
          auto pub = create_publisher<sensor_msgs::msg::CameraInfo>(topic, qos);

          publishers_[extracted_name] = pub;
          camera_infos_[extracted_name] = camera_info; // Store the loaded info using extracted name as key

        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to load calibration file: %s", file_path.c_str());
        }
      }
    }

    if (publishers_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No calibration files loaded. No CameraInfo will be published.");
        return; // No need for a timer if nothing to publish
    }

    // ----- Timer to publish loaded info -----
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), // Publish at 10 Hz
      std::bind(&CameraInfoPublisher::on_timer, this));
  }

private:
  void on_timer()
  {
    auto now = this->now();
    // Iterate through the map of loaded camera infos
    for (auto const& [name, info] : camera_infos_) {
      // Copy the stored info, update timestamp, and publish
      auto msg = info; // Make a copy
      msg.header.stamp = now;
      // Frame ID should already be set correctly during loading based on filename
      // if (msg.header.frame_id.empty()) { // This check is likely redundant now
      //   msg.header.frame_id = name; // 'name' here is the key (extracted from filename)
      // }

      // Check if publisher exists for this camera name before publishing
      if (publishers_.count(name)) {
          publishers_[name]->publish(msg);
      } else {
           RCLCPP_WARN_ONCE(this->get_logger(), "No publisher found for camera '%s', although info was loaded.", name.c_str());
      }
    }
  }

  // Use maps to associate camera name with its publisher and info
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> publishers_;
  std::map<std::string, sensor_msgs::msg::CameraInfo> camera_infos_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
