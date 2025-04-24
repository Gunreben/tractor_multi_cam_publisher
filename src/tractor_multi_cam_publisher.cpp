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
    // Create base topic publisher
    if (mode_ == OutputMode::RAW || mode_ == OutputMode::BOTH) {
      pub_raw_ = this->create_publisher<sensor_msgs::msg::Image>(
          "/camera/" + label + "/image_raw", qos);
    }
    if (mode_ == OutputMode::COMPRESSED || mode_ == OutputMode::BOTH) {
      pub_compressed_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
          "/camera/" + label + "/image_raw/compressed", qos);
    }

    // ----- GStreamer pipeline --------------------------------------------
    const std::string pipeline_str = build_pipeline(ip);
    RCLCPP_INFO(get_logger(), "Creating pipeline: %s", pipeline_str.c_str());
    
    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error) {
      RCLCPP_ERROR(get_logger(), "Pipeline parse error: %s", error->message);
      g_error_free(error);
      throw std::runtime_error("GStreamer pipeline creation failed");
    }

    if (pub_compressed_) {
      sink_jpeg_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink_jpeg"));
      if (!sink_jpeg_) {
        RCLCPP_ERROR(get_logger(), "Failed to get JPEG sink");
      } else {
        configure_sink(sink_jpeg_);
      }
    }
    if (pub_raw_) {
      sink_raw_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink_raw"));
      if (!sink_raw_) {
        RCLCPP_ERROR(get_logger(), "Failed to get raw sink");
      } else {
        configure_sink(sink_raw_);
      }
    }

    // Set pipeline to PLAYING state
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to set pipeline to playing state");
      throw std::runtime_error("Failed to start GStreamer pipeline");
    }

    // Add bus watch to catch errors
    GstBus *bus = gst_element_get_bus(pipeline_);
    gst_bus_add_watch(bus, (GstBusFunc)bus_callback, this);
    gst_object_unref(bus);

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
        "rtspsrc location=rtsp://" + ip + ":8554/h264 latency=0 buffer-mode=4 ! "
        "rtph264depay ! h264parse ! nvv4l2decoder enable-max-performance=1 ! ";

    switch (mode_) {
    case OutputMode::RAW:
      // Raw only - use I420 format which is more widely supported
      return src +
             "nvvideoconvert ! video/x-raw(memory:NVMM),format=NV12 ! "
             "queue max-size-buffers=2 leaky=downstream ! " 
             "nvvideoconvert ! video/x-raw,format=I420 ! "
             "queue max-size-buffers=2 leaky=downstream ! "
             "videoconvert ! video/x-raw,format=BGR ! "
             "appsink name=appsink_raw max-buffers=1 drop=true sync=false";
    case OutputMode::COMPRESSED:
      // Compressed only - I420 to JPEG
      return src +
             "nvvideoconvert ! video/x-raw(memory:NVMM),format=NV12 ! "
             "queue max-size-buffers=2 leaky=downstream ! "
             "nvjpegenc quality=85 ! "
             "queue max-size-buffers=2 leaky=downstream ! "
             "appsink name=appsink_jpeg max-buffers=1 drop=true sync=false";
    case OutputMode::BOTH:
    default:
      // Tee: JPEG + BGR with more robust queuing
      return src +
             "nvvideoconvert ! video/x-raw(memory:NVMM),format=NV12 ! "
             "queue max-size-buffers=2 leaky=downstream ! tee name=t "
             "t. ! queue max-size-buffers=2 leaky=downstream ! "
             "nvjpegenc quality=85 ! "
             "queue max-size-buffers=1 leaky=downstream ! "
             "appsink name=appsink_jpeg max-buffers=1 drop=true sync=false "
             "t. ! queue max-size-buffers=2 leaky=downstream ! "
             "nvvideoconvert ! video/x-raw,format=I420 ! "
             "queue max-size-buffers=1 leaky=downstream ! "
             "videoconvert ! video/x-raw,format=BGR ! "
             "appsink name=appsink_raw max-buffers=1 drop=true sync=false";
    }
  }

  static void configure_sink(GstAppSink *sink) {
    if (!sink) return;
    
    // Create caps for the appsink
    GstCaps *caps = nullptr;
    if (gst_element_get_name(GST_ELEMENT(sink)) == std::string("appsink_raw")) {
      caps = gst_caps_from_string("video/x-raw,format=BGR");
    } else {
      caps = gst_caps_from_string("image/jpeg");
    }
    
    // Configure the appsink
    gst_app_sink_set_caps(sink, caps);
    gst_app_sink_set_emit_signals(sink, false);
    gst_app_sink_set_drop(sink, true);
    gst_app_sink_set_max_buffers(sink, 1);
    
    if (caps) {
      gst_caps_unref(caps);
    }
  }

  // Bus callback to catch errors
  static gboolean bus_callback(GstBus *bus, GstMessage *message, gpointer data) {
    TractorMultiCamPublisher *self = static_cast<TractorMultiCamPublisher*>(data);
    
    switch (GST_MESSAGE_TYPE(message)) {
      case GST_MESSAGE_ERROR: {
        GError *err = nullptr;
        gchar *debug = nullptr;
        gst_message_parse_error(message, &err, &debug);
        RCLCPP_ERROR(self->get_logger(), "GStreamer error: %s", err->message);
        if (debug) {
          RCLCPP_ERROR(self->get_logger(), "Debug info: %s", debug);
        }
        g_error_free(err);
        g_free(debug);
        break;
      }
      case GST_MESSAGE_WARNING: {
        GError *err = nullptr;
        gchar *debug = nullptr;
        gst_message_parse_warning(message, &err, &debug);
        RCLCPP_WARN(self->get_logger(), "GStreamer warning: %s", err->message);
        if (debug) {
          RCLCPP_WARN(self->get_logger(), "Debug info: %s", debug);
        }
        g_error_free(err);
        g_free(debug);
        break;
      }
      default:
        break;
    }
    
    return TRUE;
  }

  // ---------- Frame processing -------------------------------------------
  void process_frames() {
    const guint64 timeout = GST_SECOND / 30;  // pull at ~30 Hz max
    RCLCPP_INFO(get_logger(), "[%s] Started in %s mode", label_.c_str(),
                (mode_ == OutputMode::RAW    ? "RAW" :
                 mode_ == OutputMode::COMPRESSED ? "COMPRESSED" : "BOTH"));

    while (rclcpp::ok() && running_) {
      try {
        if (sink_jpeg_) {
          GstSample *s = gst_app_sink_try_pull_sample(sink_jpeg_, timeout);
          if (s) {
            publish_compressed(s);
            gst_sample_unref(s);
          }
        }
        
        if (sink_raw_) {
          GstSample *s = gst_app_sink_try_pull_sample(sink_raw_, timeout);
          if (s) {
            publish_raw(s);
            gst_sample_unref(s);
          }
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Exception in process_frames: %s", e.what());
        // Add small delay to avoid tight error loops
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }

  // ---------- Publishers --------------------------------------------------
  void publish_compressed(GstSample *sample) {
    if (!pub_compressed_ || !sample) return;
    
    GstBuffer *buf = gst_sample_get_buffer(sample);
    if (!buf) {
      RCLCPP_WARN(get_logger(), "Empty buffer in compressed sample");
      return;
    }
    
    GstMapInfo map{};
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
      RCLCPP_WARN(get_logger(), "Failed to map buffer");
      return;
    }

    try {
      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header.stamp = this->now();
      msg->header.frame_id = "camera_" + label_;
      msg->format = "jpeg";
      msg->data.assign(map.data, map.data + map.size);

      pub_compressed_->publish(std::move(msg));
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception in publish_compressed: %s", e.what());
    }
    
    gst_buffer_unmap(buf, &map);
  }

  void publish_raw(GstSample *sample) {
    if (!pub_raw_ || !sample) return;
    
    GstBuffer *buf = gst_sample_get_buffer(sample);
    if (!buf) {
      RCLCPP_WARN(get_logger(), "Empty buffer in raw sample");
      return;
    }
    
    GstMapInfo map{};
    if (!gst_buffer_map(buf, &map, GST_MAP_READ)) {
      RCLCPP_WARN(get_logger(), "Failed to map buffer");
      return;
    }

    try {
      GstCaps *caps = gst_sample_get_caps(sample);
      GstStructure *st = gst_caps_get_structure(caps, 0);
      int w = 0, h = 0;
      gst_structure_get_int(st, "width", &w);
      gst_structure_get_int(st, "height", &h);

      if (w <= 0 || h <= 0) {
        RCLCPP_WARN(get_logger(), "Invalid dimensions: %dx%d", w, h);
      } else {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_" + label_;
        msg->height = static_cast<uint32_t>(h);
        msg->width = static_cast<uint32_t>(w);
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = w * 3;
        msg->data.assign(map.data, map.data + map.size);

        pub_raw_->publish(std::move(msg));
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Exception in publish_raw: %s", e.what());
    }
    
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
      try {
        auto node = std::make_shared<TractorMultiCamPublisher>(ip, label);
        rclcpp::spin(node);
      } catch (const std::exception &e) {
        std::cerr << "Exception in thread for " << label << ": " << e.what() << std::endl;
      }
    });
  }

  for (auto &t : threads) {
    if (t.joinable()) {
      t.join();
    }
  }
  
  rclcpp::shutdown();
  return 0;
}
