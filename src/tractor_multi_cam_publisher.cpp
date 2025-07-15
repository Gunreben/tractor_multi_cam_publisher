#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/app.h>
#include <memory>
#include <vector>
#include <thread>
#include <string>
#include <atomic>

/**
 * TractorMultiCamPublisher – publishes camera frames using image_transport.
 *
 * This node uses a GStreamer pipeline to decode an RTSP stream and publishes
 * the resulting frames. By using image_transport, it offers both raw and
 * compressed topics automatically from a single publisher. The pipeline uses
 * hardware-accelerated elements (`nvv4l2decoder`, `nvvideoconvert`, `nvjpegenc`)
 * available on NVIDIA Jetson platforms.
 *
 * The output encoding is BGR8, which is compatible with RViz2 and other
 * standard ROS 2 tools.
 */
class TractorMultiCamPublisher : public rclcpp::Node {
public:
  TractorMultiCamPublisher(const std::string &ip, const std::string &label)
      : Node("tractor_multi_cam_publisher_" + label), label_(label) {
    
    // ----- QoS Profile ----------------------------------------------------
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    // ----- Image Transport Publisher --------------------------------------
    // Create an ImageTransport instance and advertise the base topic.
    // image_transport will automatically create publishers for raw and compressed topics.
    it_ = std::make_shared<image_transport::ImageTransport>(
        std::shared_ptr<rclcpp::Node>(this, [](auto) {})); // Aliasing constructor
    pub_ = it_->advertise("/camera/" + label_ + "/image_raw", qos.get_rmw_qos_profile());

    // ----- GStreamer pipeline ---------------------------------------------
    const std::string pipeline_str = build_pipeline(ip);
    RCLCPP_INFO(get_logger(), "[%s] Creating pipeline: %s", label_.c_str(), pipeline_str.c_str());
    
    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
    if (error) {
      RCLCPP_ERROR(get_logger(), "Pipeline parse error: %s", error->message);
      g_error_free(error);
      throw std::runtime_error("GStreamer pipeline creation failed");
    }

    appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "appsink"));
    if (!appsink_) {
        RCLCPP_ERROR(get_logger(), "Failed to get appsink element");
        throw std::runtime_error("Could not get GStreamer appsink");
    }
    configure_sink(appsink_);

    // ----- Start Pipeline and Processing Thread ---------------------------
    GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING state");
      throw std::runtime_error("Failed to start GStreamer pipeline");
    }

    GstBus *bus = gst_element_get_bus(pipeline_);
    gst_bus_add_watch(bus, (GstBusFunc)bus_callback, this);
    gst_object_unref(bus);

    processing_thread_ = std::thread(&TractorMultiCamPublisher::process_frames, this);
    RCLCPP_INFO(get_logger(), "[%s] Node started and publishing.", label_.c_str());
  }

  ~TractorMultiCamPublisher() override {
    running_ = false;
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (pipeline_) {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        gst_object_unref(pipeline_);
    }
  }

private:
  // ---------- GStreamer Helpers ------------------------------------------
  std::string build_pipeline(const std::string &ip) const {
    // This pipeline decodes the H.264 stream, converts it to BGR format,
    // and sends it to a single appsink. image_transport handles compression.
    // ":8554/h264 latency=100 drop-on-latency=true ! "
	return "rtspsrc location=rtsp://" + ip + ":8554/h264  latency=200 drop-on-latency=false  !"
       "rtph264depay ! h264parse ! queue ! "
       "nvv4l2decoder ! queue ! "
       "nvvidconv ! video/x-raw,format=BGRx ! "
       "videoconvert ! video/x-raw,format=BGR ! "
       "appsink name=appsink max-buffers=1 drop=true sync=false";

  }

  void configure_sink(GstAppSink *sink) {
    // Set the appsink to output BGR frames, which image_transport can then
    // use for both raw and compressed publishing.
    GstCaps *caps = gst_caps_new_simple("video/x-raw",
                                        "format", G_TYPE_STRING, "BGR",
                                        NULL);
    gst_app_sink_set_caps(sink, caps);
    gst_caps_unref(caps);
  }

  static gboolean bus_callback(GstBus *bus, GstMessage *message, gpointer data) {
    auto self = static_cast<TractorMultiCamPublisher*>(data);
    GError *err = nullptr;
    gchar *debug_info = nullptr;
    
    switch (GST_MESSAGE_TYPE(message)) {
      case GST_MESSAGE_ERROR:
        gst_message_parse_error(message, &err, &debug_info);
        RCLCPP_ERROR(self->get_logger(), "GStreamer Error: %s. Debug info: %s",
                     err->message, debug_info ? debug_info : "none");
        g_clear_error(&err);
        g_free(debug_info);
        break;
      case GST_MESSAGE_WARNING:
        gst_message_parse_warning(message, &err, &debug_info);
        RCLCPP_WARN(self->get_logger(), "GStreamer Warning: %s. Debug info: %s",
                    err->message, debug_info ? debug_info : "none");
        g_clear_error(&err);
        g_free(debug_info);
        break;
      default:
        break;
    }
    return TRUE;
  }

  // ---------- Frame Processing and Publishing ----------------------------
  void process_frames() {
    while (rclcpp::ok() && running_) {
      GstSample *sample = gst_app_sink_pull_sample(appsink_);

      if (sample) {
        try {
          publish_frame(sample);
        } catch (const std::exception &e) {
          RCLCPP_ERROR(get_logger(), "Exception in process_frames: %s", e.what());
        }
        gst_sample_unref(sample);
      } else {
        // This can happen on shutdown or if the stream stops
        if (running_) {
            RCLCPP_WARN(get_logger(), "gst_app_sink_pull_sample() returned null");
        }
      }
    }
  }

  void publish_frame(GstSample *sample) {
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) return;

    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ)) return;
    
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *s = gst_caps_get_structure(caps, 0);
    int width = 0, height = 0;
    gst_structure_get_int(s, "width", &width);
    gst_structure_get_int(s, "height", &height);

    if (width > 0 && height > 0) {
        sensor_msgs::msg::Image msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "camera_" + label_;
        msg.height = height;
        msg.width = width;
        msg.encoding = "bgr8";
        msg.is_bigendian = false;
        msg.step = width * 3;
        msg.data.assign(map_info.data, map_info.data + map_info.size);

        pub_.publish(msg);
    }
    
    gst_buffer_unmap(buffer, &map_info);
  }

  // ---------- Members -----------------------------------------------------
  std::string label_;
  
  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher pub_;

  GstElement *pipeline_ = nullptr;
  GstAppSink *appsink_  = nullptr;

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

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<std::shared_ptr<TractorMultiCamPublisher>> nodes;

  for (const auto &[ip, label] : cameras) {
    try {
      auto node = std::make_shared<TractorMultiCamPublisher>(ip, label);
      nodes.push_back(node);
      executor.add_node(node);
    } catch (const std::exception &e) {
      std::cerr << "Failed to create node for camera " << label << ": " << e.what() << std::endl;
    }
  }
  
  if (!nodes.empty()) {
    executor.spin();
  }

  rclcpp::shutdown();
  gst_deinit();
  return 0;
}
