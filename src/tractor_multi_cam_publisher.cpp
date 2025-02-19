#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <gst/gst.h>
#include <gst/app/app.h>
#include <memory>
#include <vector>
#include <thread>
#include <utility>

class TractorMultiCamPublisher : public rclcpp::Node {
public:
  TractorMultiCamPublisher(const std::string& ip, const std::string& label) 
    : Node("tractor_multi_cam_publisher_" + label),
    label_(label)   {
    
    // QoS Configuration
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .best_effort()
      .durability_volatile();

    // Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/" + label + "/compressed", qos);

    // GStreamer Pipeline Setup for H.264
    std::string pipeline_str = 
      "rtspsrc location=rtsp://" + ip + ":8554/h264 latency=0 ! "
      "rtph264depay ! h264parse ! nvv4l2decoder ! "
      "nvvidconv ! video/x-raw,format=I420 ! "
      "nvjpegenc ! appsink name=appsink";

    GError* error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error) {
      RCLCPP_ERROR(get_logger(), "Pipeline error: %s", error->message);
      g_error_free(error);
      return;
    }

    appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink"));
    gst_app_sink_set_emit_signals(appsink_, true);
    gst_app_sink_set_drop(appsink_, true);

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    processing_thread_ = std::thread(&TractorMultiCamPublisher::process_frames, this);
  }

  ~TractorMultiCamPublisher() {
    running_ = false;
    if (processing_thread_.joinable()) processing_thread_.join();
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }

private:
  std::string label_;
  void process_frames() {
    while (rclcpp::ok() && running_) {
      GstSample* sample = gst_app_sink_try_pull_sample(appsink_, GST_MSECOND);
      if (!sample) continue;

      GstBuffer* buffer = gst_sample_get_buffer(sample);
      GstMapInfo map;
      if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->format = "jpeg";

        msg->header.stamp = this->now();
        msg->header.frame_id = "camera/" + label_; 

        msg->data.assign(map.data, map.data + map.size);
        publisher_->publish(std::move(msg));
        gst_buffer_unmap(buffer, &map);
      }
      gst_sample_unref(sample);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
  GstElement* pipeline_;
  GstAppSink* appsink_;
  std::thread processing_thread_;
  bool running_ = true;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  gst_init(&argc, &argv);
  
  const std::vector<std::pair<std::string, std::string>> cameras = {
    {"192.168.26.70", "side_left"},
    {"192.168.26.71", "rear_left"},
    {"192.168.26.72", "rear_mid"},
    {"192.168.26.73", "rear_right"},
    {"192.168.26.74", "side_right"}
  };

  std::vector<std::thread> threads;
  for (const auto& [ip, label] : cameras) {
    threads.emplace_back([ip, label]() {
      rclcpp::spin(std::make_shared<TractorMultiCamPublisher>(ip, label));
    });
  }

  for (auto& t : threads) t.join();
  rclcpp::shutdown();
  return 0;
}
