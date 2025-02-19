#include <chrono>
#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;

class CameraInfoPublisher : public rclcpp::Node
{
public:
  CameraInfoPublisher()
  : Node("camera_info_publisher")
  {
    // Camera names (adjust or add as needed):
    camera_names_ = {
      "side_left",
      "rear_left",
      "rear_mid",
      "rear_right",
      "side_right"
    };

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
    .best_effort()
    .durability_volatile();

    // Create one publisher per camera, publishing to /camera/<name>/camera_info
    for (const auto &name : camera_names_) {
      std::string topic = "/camera/" + name + "/camera_info";

      auto pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(topic, qos);
      publishers_.push_back(pub);
      //RCLCPP_INFO(this->get_logger(), "Publisher created for: %s", topic.c_str());
    }

    // Set up the static CameraInfo message (intrinsics, distortion, etc.)
    setupCameraInfo();

    // Create a timer to publish once per second
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&CameraInfoPublisher::timerCallback, this));
  }

private:
  void setupCameraInfo()
  {
    // For 1280x720 with a ~130° horizontal FOV => fx ~ 298 (assuming pinhole & square pixels)
    double fx = 298.0;
    double fy = 298.0;
    double cx = 640.0;  // half of width = 640
    double cy = 360.0;  // half of height = 360

    camera_info_.width = 1280;
    camera_info_.height = 720;
    camera_info_.distortion_model = "plumb_bob";  // or "equidistant" if you're modeling fisheye
    
    // Distortion coefficients (k1, k2, t1, t2, k3) - set to 0 for a placeholder
    camera_info_.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // Intrinsic camera matrix (3x3)
    // [ fx   0  cx ]
    // [  0  fy  cy ]
    // [  0   0   1 ]
    camera_info_.k = {
      fx,   0.0,  cx,
      0.0,  fy,   cy,
      0.0,  0.0,  1.0
    };

    // Rectification matrix (identity if no stereo or other rotation)
    camera_info_.r = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    // Projection matrix (3x4)
    // [ fx  0   cx  0 ]
    // [ 0  fy   cy  0 ]
    // [ 0   0    1  0 ]
    camera_info_.p = {
      fx,   0.0,  cx,   0.0,
      0.0,  fy,   cy,   0.0,
      0.0,  0.0,  1.0,  0.0
    };
  }

  void timerCallback()
  {
    // Update timestamp each time we publish
    camera_info_.header.stamp = this->now();

    // Publish on each camera topic
    for (size_t i = 0; i < publishers_.size(); ++i) {
      // Give each camera_info a unique frame_id
      camera_info_.header.frame_id = "camera_" + camera_names_[i] + "_frame";

      publishers_[i]->publish(camera_info_);
      /*RCLCPP_INFO(
        this->get_logger(),
        "Published camera info on /camera/%s/camera_info",
        camera_names_[i].c_str()
      );*/
    }
  }

  std::vector<std::string> camera_names_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> publishers_;
  sensor_msgs::msg::CameraInfo camera_info_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraInfoPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
