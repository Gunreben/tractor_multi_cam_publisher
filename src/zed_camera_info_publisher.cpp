// src/zed_camera_info_publisher.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class ZedCameraInfoPublisher : public rclcpp::Node
{
public:
  ZedCameraInfoPublisher()
  : Node("zed_camera_info_publisher")
  {
    // QoS settings
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .best_effort()
                   .durability_volatile();

    // Create publisher
    publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/zed/left/camera_info", qos);

    // Initialize camera info with provided values
    camera_info_.header.frame_id = "zed_left_camera_optical_frame";
    camera_info_.height = 360;
    camera_info_.width = 640;
    camera_info_.distortion_model = "rational_polynomial";
    
    // Distortion coefficients (d)
    camera_info_.d = {
        -1.0929299592971802,
        2.671180009841919,
        -0.0002514079969841987,
        -4.404849823913537e-05,
        0.10725899785757065,
        -0.9946709871292114,
        2.5193400382995605,
        0.27934199571609497
    };
    
    // Camera matrix (k)
    camera_info_.k = {
        267.44000244140625, 0.0, 317.864990234375,
        0.0, 267.5299987792969, 174.08250427246094,
        0.0, 0.0, 1.0
    };
    
    // Rectification matrix (r)
    camera_info_.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    
    // Projection matrix (p)
    camera_info_.p = {
        267.44000244140625, 0.0, 317.864990234375, 0.0,
        0.0, 267.5299987792969, 174.08250427246094, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    
    // Binning
    camera_info_.binning_x = 0;
    camera_info_.binning_y = 0;
    
    // ROI
    camera_info_.roi.x_offset = 0;
    camera_info_.roi.y_offset = 0;
    camera_info_.roi.height = 0;
    camera_info_.roi.width = 0;
    camera_info_.roi.do_rectify = false;

    // Timer to publish at 10 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ZedCameraInfoPublisher::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "ZED camera info publisher initialized");
  }

private:
  void on_timer()
  {
    // Update timestamp
    camera_info_.header.stamp = this->now();
    publisher_->publish(camera_info_);
  }

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr publisher_;
  sensor_msgs::msg::CameraInfo camera_info_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedCameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}

