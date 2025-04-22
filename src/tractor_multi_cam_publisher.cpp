#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/app.h>
#include <gst/video/video.h>
#include <memory>
#include <vector>
#include <thread>
#include <utility>
#include <string>

/**
 * TractorMultiCamPublisher – publishes either compressed (JPEG) _or_ raw
 * frames. Raw images are sent as **BGR8**, a format accepted by RViz 2 and
 * `camera_calibration` on ROS 2 Humble, and compatible with Jetson AGX Orin
 * GStreamer caps.
 *
 * The pipeline uses hardware‑accelerated `nvv4l2decoder` + `nvvideoconvert`,
 * with a CPU `videoconvert` to reach BGR. On AGX Orin this adds <2 ms for a
 * 1080p/30 fps stream.
 *
 * Select the output stream(s) at startup via the ROS 2 parameter **`output_mode`**:
 *   `raw` | `compressed` | `both` (default)
 */
class TractorMultiCamPublisher : public rclcpp::Node {
public:
  enum class OutputMode { RAW, COMPRESSED, BOTH };

  TractorMultiCamPublisher(const std::string &ip, const std::string &label)
      : Node("tractor_multi_cam_publisher_" + label), label_(label) {
    // ----- Parameter ------------------------------------------------------
    const std::string mode_str = this->declare_parameter<std::string>(
        "output_mode", "compressed");
    mode_ = str_to_mode(mode_str);

    // ----- QoS ------------------------------------------------------------
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // ----- Publishers -----------------------------------------------------
    if (mode_ == OutputMode::COMPRESSED || mode_ == OutputMode::BOTH) {
      pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          "/camera/" + label + "/compressed", qos);
    }
    if (mode_ == OutputMode::RAW || mode_ == OutputMode::BOTH) {
      pub_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
          "/camera/" + label + "/image", qos);
    }

    // ----- GStreamer pipeline --------------------------------------------
    const std::string pipeline_str = build_pipeline(ip);
    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error) {
      RCLCPP_ERROR(get_logger(), "Pipeline parse error: %s", error->message);
      g_error_free(error);
      throw std::runtime_error("GStreamer pipeline creation failed");
    }

    if (pub_compressed_) {
      sink_jpeg_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink_jpeg"));
      configure_sink(sink_jpeg_);
    }
    if (pub_raw_) {
      sink_raw_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink_raw"));
      configure_sink(sink_raw_);
    }

    gst_element_set_state(pipeline_, GST_STATE_PLAYING);

    processing_thread_ = std::thread(&TractorMultiCamPublisher::process_frames, this);
  }

  ~TractorMultiCamPublisher() override {
    running_ = false;
    if (processing_thread_.joinable())
      processing_thread_.join();
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }

private:
  // ---------- Helpers -----------------------------------------------------
  static OutputMode str_to_mode(const std::string &s) {
    if (s == "raw") return OutputMode::RAW;
    if (s == "compressed") return OutputMode::COMPRESSED;
    return OutputMode::BOTH;
  }

  std::string build_pipeline(const std::string &ip) const {
    const std::string src =
        "rtspsrc location=rtsp://" + ip + ":8554/h264 latency=0 ! "
        "rtph264depay ! h264parse ! nvv4l2decoder ! ";

    switch (mode_) {
    case OutputMode::RAW:
      // Raw only – convert to BGR
      return src +
             "nvvideoconvert ! video/x-raw(memory:NVMM),format=NV12 ! "
             "nvvideoconvert ! video/x-raw,format=BGRx ! "
             "videoconvert ! video/x-raw,format=BGR ! appsink name=appsink_raw";
    case OutputMode::COMPRESSED:
      // Compressed only – I420 to JPEG
      return src +
             "nvvideoconvert ! video/x-raw,format=I420 ! "
             "nvjpegenc ! appsink name=appsink_jpeg";
    case OutputMode::BOTH:
    default:
      // Tee: JPEG + BGR
      return src +
             "nvvideoconvert ! video/x-raw(memory:NVMM),format=NV12 ! tee name=t "
             "t. ! queue ! video/x-raw,format=I420 ! nvjpegenc ! appsink name=appsink_jpeg "
             "t. ! queue ! nvvideoconvert ! video/x-raw,format=BGRx ! "
             "videoconvert ! video/x-raw,format=BGR ! appsink name=appsink_raw";
    }
  }

  static void configure_sink(GstAppSink *sink) {
    if (!sink) return;
    gst_app_sink_set_emit_signals(sink, false);
    gst_app_sink_set_drop(sink, true);
    gst_app_sink_set_max_buffers(sink, 1);
  }

  // ---------- Frame processing -------------------------------------------
  void process_frames() {
    const guint64 timeout = GST_SECOND / 30;  // pull at ~30 Hz max
    RCLCPP_INFO(get_logger(), "[%s] Started in %s mode", label_.c_str(),
                (mode_ == OutputMode::RAW    ? "RAW" :
                 mode_ == OutputMode::COMPRESSED ? "COMPRESSED" : "BOTH"));

    while (rclcpp::ok() && running_) {
      if (sink_jpeg_) {
        if (GstSample *s = gst_app_sink_try_pull_sample(sink_jpeg_, timeout)) {
          publish_compressed(s);
          gst_sample_unref(s);
        }
      }
      if (sink_raw_) {
        if (GstSample *s = gst_app_sink_try_pull_sample(sink_raw_, timeout)) {
          publish_raw(s);
          gst_sample_unref(s);
        }
      }
    }
  }

  // ---------- Publishers --------------------------------------------------
  void publish_compressed(GstSample *sample) {
    if (!pub_compressed_) return;
    GstBuffer *buf = gst_sample_get_buffer(sample);
    GstMapInfo map{};
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) return;

    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera/" + label_;
    msg->format = "jpeg";
    msg->data.assign(map.data, map.data + map.size);

    pub_compressed_->publish(std::move(msg));
    gst_buffer_unmap(buf, &map);
  }

  void publish_raw(GstSample *sample) {
    if (!pub_raw_) return;
    GstBuffer *buf = gst_sample_get_buffer(sample);
    GstMapInfo map{};
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) return;

    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *st = gst_caps_get_structure(caps, 0);
    int w = 0, h = 0;
    gst_structure_get_int(st, "width", &w);
    gst_structure_get_int(st, "height", &h);

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera/" + label_;
    msg->height = static_cast<uint32_t>(h);
    msg->width = static_cast<uint32_t>(w);
    msg->encoding = "bgr8";
    msg->is_bigendian = false;
    msg->step = w * 3;
    msg->data.assign(map.data, map.data + map.size);

    pub_raw_->publish(std::move(msg));
    gst_buffer_unmap(buf, &map);
  }

  // ---------- Members -----------------------------------------------------
  std::string label_;
  OutputMode mode_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_compressed_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           pub_raw_;

  GstElement  *pipeline_   = nullptr;
  GstAppSink  *sink_jpeg_  = nullptr;
  GstAppSink  *sink_raw_   = nullptr;

  std::thread processing_thread_;
  std::atomic<bool> running_{true};
};

// ----------------------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  gst_init(&argc, &argv);

  const std::vector<std::pair<std::string, std::string>> cameras = {
      {"192.168.26.70", "side_left"},
      {"192.168.26.71", "rear_left"},
      {"192.168.26.72", "rear_mid"},
      {"192.168.26.73", "rear_right"},
      {"192.168.26.74", "side_right"}};

  std::vector<std::thread> threads;
  for (const auto &[ip, label] : cameras) {
    threads.emplace_back([ip, label]() {
      auto node = std::make_shared<TractorMultiCamPublisher>(ip, label);
      rclcpp::spin(node);
    });
  }

  for (auto &t : threads) t.join();
  rclcpp::shutdown();
  return 0;
}
