// src/camera_info_publisher.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class CameraInfoPublisher : public rclcpp::Node
{
public:
  CameraInfoPublisher()
  : Node("camera_info_publisher")
  {
    // camera names:
    camera_names_ = {
      "side_left",
      "rear_left",
      "rear_mid",
      "rear_right",
      "side_right"
    };

    // QoS: match a typical camera_info (drops okay, no history)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .best_effort()
                   .durability_volatile();

    // Pre-build publishers and static CameraInfo messages
    for (const auto & name : camera_names_) {
      // topic: /camera/<name>/camera_info
      auto topic = std::string("/camera/") + name + "/camera_info";
      auto pub = create_publisher<sensor_msgs::msg::CameraInfo>(topic, qos);
      publishers_.push_back(pub);

      sensor_msgs::msg::CameraInfo info;
      info.header.frame_id = name;            // frame_id = camera name
      info.height            = 720;
      info.width             = 1280;
      info.distortion_model  = "plumb_bob";   // typical model

      // distortion coefficients: k1, k2, p1, p2
      info.d = {-0.019592, -0.056248, 0.075002, -0.043589};

      // camera matrix K (row-major)
      info.k = {
        638.006404, 1.059101, 633.759633,
        0.0,        638.981696, 369.883738,
        0.0,        0.0,         1.0
      };

      // rectification matrix R
      info.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
      };

      // projection matrix P (3×4)
      info.p = {
        638.006404, 1.059101, 633.759633, 0.0,
        0.0,        638.981696, 369.883738, 0.0,
        0.0,        0.0,         1.0,        0.0
      };

      camera_infos_.push_back(info);
    }

    // publish at 10 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CameraInfoPublisher::on_timer, this));
  }

private:
  void on_timer()
  {
    auto now = this->now();
    for (size_t i = 0; i < publishers_.size(); ++i) {
      auto msg = camera_infos_[i];
      msg.header.stamp = now;
      publishers_[i]->publish(msg);
    }
  }

  std::vector<std::string> camera_names_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> publishers_;
  std::vector<sensor_msgs::msg::CameraInfo> camera_infos_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
