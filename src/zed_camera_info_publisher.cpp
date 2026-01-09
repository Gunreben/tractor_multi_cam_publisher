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

    // Create publishers
    left_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/zed/left/camera_info", qos);
    right_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "/zed/right/camera_info", qos);

    // Initialize LEFT camera info
    left_camera_info_.header.frame_id = "zed_left_camera_optical_frame";
    left_camera_info_.height = 360;
    left_camera_info_.width = 640;
    left_camera_info_.distortion_model = "rational_polynomial";
    
    // Left distortion coefficients (d)
    left_camera_info_.d = {
        -1.0929299592971802,
        2.671180009841919,
        -0.0002514079969841987,
        -4.404849823913537e-05,
        0.10725899785757065,
        -0.9946709871292114,
        2.5193400382995605,
        0.27934199571609497
    };
    
    // Left camera matrix (k)
    left_camera_info_.k = {
        267.44000244140625, 0.0, 317.864990234375,
        0.0, 267.5299987792969, 174.08250427246094,
        0.0, 0.0, 1.0
    };
    
    // Left rectification matrix (r)
    left_camera_info_.r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    
    // Left projection matrix (p)
    left_camera_info_.p = {
        267.44000244140625, 0.0, 317.864990234375, 0.0,
        0.0, 267.5299987792969, 174.08250427246094, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    
    // Left binning
    left_camera_info_.binning_x = 0;
    left_camera_info_.binning_y = 0;
    
    // Left ROI
    left_camera_info_.roi.x_offset = 0;
    left_camera_info_.roi.y_offset = 0;
    left_camera_info_.roi.height = 0;
    left_camera_info_.roi.width = 0;
    left_camera_info_.roi.do_rectify = false;

    // Initialize RIGHT camera info
    right_camera_info_.header.frame_id = "zed_right_camera_optical_frame";
    right_camera_info_.height = 621;
    right_camera_info_.width = 1104;
    right_camera_info_.distortion_model = "rational_polynomial";
    
    // Right distortion coefficients (d)
    right_camera_info_.d = {
        -1.2673100233078003,
        2.8115999698638916,
        -0.0003399539855308831,
        -8.335019811056554e-05,
        0.06889919936656952,
        -1.173509955406189,
        2.6618800163269043,
        0.23963700234889984
    };
    
    // Right camera matrix (k)
    right_camera_info_.k = {
        534.1099853515625, 0.0, 551.760009765625,
        0.0, 534.1799926757812, 316.76849365234375,
        0.0, 0.0, 1.0
    };
    
    // Right rectification matrix (r)
    right_camera_info_.r = {
        0.9999848008155823, 0.004049811512231827, -0.0037440864834934473,
        -0.004061334766447544, 0.999987006187439, -0.003075310029089451,
        0.0037315834779292345, 0.003090469166636467, 0.9999882578849792
    };
    
    // Right projection matrix (p)
    right_camera_info_.p = {
        534.1099853515625, 0.0, 551.760009765625, -64.20011901855469,
        0.0, 534.1799926757812, 316.76849365234375, 0.0,
        0.0, 0.0, 1.0, 0.0
    };
    
    // Right binning
    right_camera_info_.binning_x = 0;
    right_camera_info_.binning_y = 0;
    
    // Right ROI
    right_camera_info_.roi.x_offset = 0;
    right_camera_info_.roi.y_offset = 0;
    right_camera_info_.roi.height = 0;
    right_camera_info_.roi.width = 0;
    right_camera_info_.roi.do_rectify = false;

    // Timer to publish at 10 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ZedCameraInfoPublisher::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "ZED camera info publisher initialized (left & right)");
  }

private:
  void on_timer()
  {
    // Update timestamps
    auto now = this->now();
    left_camera_info_.header.stamp = now;
    right_camera_info_.header.stamp = now;
    
    // Publish both camera infos
    left_publisher_->publish(left_camera_info_);
    right_publisher_->publish(right_camera_info_);
  }

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_publisher_;
  sensor_msgs::msg::CameraInfo left_camera_info_;
  sensor_msgs::msg::CameraInfo right_camera_info_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedCameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}

