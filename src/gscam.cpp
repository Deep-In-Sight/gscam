// Copyright 2022 Jonathan Bohren, Clyde McQueen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdlib.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>

extern "C" {
#include "gst/gst.h"
#include "gst/app/gstappsink.h"
}

#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include "gscam/gscam.hpp"

namespace gscam
{

GSCam::GSCam(const rclcpp::NodeOptions & options)
: rclcpp::Node("gscam_publisher", options),
  gsconfig_(""),
  pipeline_(nullptr),
  sink_(nullptr),
  camera_info_manager_(this),
  stop_signal_(false)
{
  if (!configure() || !init_stream()) {
    RCLCPP_FATAL(get_logger(), "Failed to configure or initialize the stream!");
    rclcpp::shutdown();
    return;
  }
  start_pipeline();
}

GSCam::~GSCam() 
{
  stop_signal_ = true;
  cleanup_stream();
}

bool GSCam::configure()
{
  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  bool gsconfig_rosparam_defined = false;
  char * gsconfig_env = nullptr;
  const auto gsconfig_rosparam = declare_parameter("gscam_config", "");
  gsconfig_rosparam_defined = !gsconfig_rosparam.empty();
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (!gsconfig_env && !gsconfig_rosparam_defined) {
    RCLCPP_FATAL(get_logger(),
      "Neither GSCAM_CONFIG environment variable nor 'gscam_config' rosparam is set.");
    return false;
  } else if (gsconfig_env && gsconfig_rosparam_defined) {
    RCLCPP_FATAL(get_logger(),
      "Both GSCAM_CONFIG environment variable and 'gscam_config' rosparam are set. Define only one.");
    return false;
  } else if (gsconfig_env) {
    gsconfig_ = gsconfig_env;
    RCLCPP_INFO_STREAM(get_logger(), "Using gstreamer config from env: \"" << gsconfig_env << "\"");
  } else {
    gsconfig_ = gsconfig_rosparam;
    RCLCPP_INFO_STREAM(get_logger(), "Using gstreamer config from rosparam: \"" << gsconfig_rosparam << "\"");
  }

  // Get additional gscam configuration
  sync_sink_ = declare_parameter("sync_sink", true);
  preroll_ = declare_parameter("preroll", false);
  use_gst_timestamps_ = declare_parameter("use_gst_timestamps", false);
  reopen_on_eof_ = declare_parameter("reopen_on_eof", false);

  // Get the camera parameters file
  camera_info_url_ = declare_parameter("camera_info_url", "");
  camera_name_ = declare_parameter("camera_name", "");

  // Get the image encoding
  image_encoding_ =
    declare_parameter("image_encoding", std::string(sensor_msgs::image_encodings::RGB8));
  if (image_encoding_ != sensor_msgs::image_encodings::RGB8 &&
      image_encoding_ != sensor_msgs::image_encodings::MONO8 &&
      image_encoding_ != sensor_msgs::image_encodings::YUV422 &&
      image_encoding_ != "jpeg")
  {
    RCLCPP_FATAL_STREAM(get_logger(), "Unsupported image encoding: " << image_encoding_);
    return false;
  }

  camera_info_manager_.setCameraName(camera_name_);
  if (camera_info_manager_.validateURL(camera_info_url_)) {
    camera_info_manager_.loadCameraInfo(camera_info_url_);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded camera calibration from " << camera_info_url_);
  } else {
    RCLCPP_WARN_STREAM(get_logger(),
      "Camera info not found at " << camera_info_url_ << ". Using uncalibrated config.");
  }

  // Get TF Frame
  frame_id_ = declare_parameter("frame_id", "camera_frame");
  if (frame_id_ == "camera_frame") {
    RCLCPP_WARN_STREAM(get_logger(),
      "No camera frame_id set, using default: " << frame_id_);
  }

  use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", false);
  return true;
}

bool GSCam::init_stream()
{
  if (!gst_is_initialized()) {
    RCLCPP_DEBUG(get_logger(), "Initializing gstreamer...");
    gst_init(nullptr, nullptr);
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer Version: " << gst_version_string());

  GError * error = nullptr;
  pipeline_ = gst_parse_launch(gsconfig_.c_str(), &error);
  if (!pipeline_) {
    RCLCPP_FATAL_STREAM(get_logger(), "GStreamer pipeline error: " << error->message);
    return false;
  }

  sink_ = gst_element_factory_make("appsink", "sink");
  if (!sink_) {
    RCLCPP_FATAL(get_logger(), "Failed to create appsink element.");
    return false;
  }

  GstCaps * caps = nullptr;
  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "RGB", NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "GRAY8", NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "UYVY", NULL);
  } else if (image_encoding_ == "jpeg") {
    caps = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }
  gst_app_sink_set_caps(GST_APP_SINK(sink_), caps);
  gst_caps_unref(caps);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be
  // dropped
  gst_base_sink_set_sync(GST_BASE_SINK(sink_), sync_sink_ ? TRUE : FALSE);

  // 파이프라인 내에 appsink 연결
  if (GST_IS_PIPELINE(pipeline_)) {
    GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    if (!outpad) {
      RCLCPP_FATAL(get_logger(), "No unlinked src pad found in pipeline.");
      return false;
    }
    GstElement * outelement = gst_pad_get_parent_element(outpad);
    gst_object_unref(outpad);
    if (!gst_bin_add(GST_BIN(pipeline_), sink_)) {
      RCLCPP_FATAL(get_logger(), "Failed to add sink to pipeline.");
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }
    if (!gst_element_link(outelement, sink_)) {
      RCLCPP_FATAL(get_logger(), "Failed to link outelement with sink.");
      gst_object_unref(outelement);
      gst_object_unref(pipeline_);
      return false;
    }
    gst_object_unref(outelement);
  } else {
    GstElement * launchpipe = pipeline_;
    pipeline_ = gst_pipeline_new(nullptr);
    gst_object_unparent(GST_OBJECT(launchpipe));
    gst_bin_add_many(GST_BIN(pipeline_), launchpipe, sink_, NULL);
    if (!gst_element_link(launchpipe, sink_)) {
      RCLCPP_FATAL(get_logger(), "Failed to link launchpipe with sink.");
      gst_object_unref(pipeline_);
      return false;
    }
  }

  // Calibration between ros::Time and gst timestamps
  GstClock * clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = now().nanoseconds() - GST_TIME_AS_NSECONDS(ct);
  RCLCPP_INFO(get_logger(), "Time offset: %.6f", rclcpp::Time(time_offset_).seconds());

  // 초기 상태를 PAUSED로 설정
  gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(), "Failed to pause stream; check GStreamer configuration.");
    return false;
  }
  RCLCPP_DEBUG(get_logger(), "Stream is PAUSED.");

  // Create ROS camera interface
  auto qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS{1};
  if (image_encoding_ == "jpeg") {
    jpeg_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "camera/image_raw/compressed", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera/camera_info", qos);
  } else {
    camera_pub_ = image_transport::create_camera_publisher(
      this, "camera/image_raw", qos.get_rmw_qos_profile());
  }
  return true;
}

  // 3. Register asynchronous callback in appsink and switch pipeline to PLAYING state
void GSCam::start_pipeline()
{
  // Pre-roll camera if needed
  if (preroll_) {
    RCLCPP_DEBUG(get_logger(), "Performing preroll...");
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PLAY during preroll.");
      return;
    }
    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
    if (gst_element_get_state(pipeline_, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PAUSE after preroll.");
      return;
    }
    RCLCPP_INFO(get_logger(), "Preroll complete; stream is PAUSED.");
  }
  GstAppSinkCallbacks callbacks = {};
  callbacks.eos = gst_eos_cb;
  callbacks.new_preroll = gst_new_preroll_cb;
  callbacks.new_sample = gst_new_sample_cb;
  gst_app_sink_set_callbacks(GST_APP_SINK(sink_), &callbacks, this, nullptr);

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Could not start stream!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Started stream asynchronously.");
}


GstFlowReturn GSCam::on_new_sample(GstAppSink * appsink)
{
  GstSample * sample = gst_app_sink_pull_sample(appsink);
  if (!sample) {
    RCLCPP_ERROR(get_logger(), "Failed to pull sample from appsink.");
    return GST_FLOW_ERROR;
  }
  GstBuffer * buf = gst_sample_get_buffer(sample);
  if (!buf) {
    gst_sample_unref(sample);
    RCLCPP_ERROR(get_logger(), "Failed to get buffer from sample.");
    return GST_FLOW_ERROR;
  }
  GstMemory * memory = gst_buffer_get_memory(buf, 0);
  GstMapInfo info;
  if (!gst_memory_map(memory, &info, GST_MAP_READ)) {
    gst_sample_unref(sample);
    RCLCPP_ERROR(get_logger(), "Failed to map memory.");
    return GST_FLOW_ERROR;
  }
  gsize buf_size = info.size;
  guint8 * buf_data = info.data;

  // 타임스탬프 계산 (gst 버퍼의 pts와 base_time, 보정값 활용)
  GstClockTime bt = gst_element_get_base_time(pipeline_);
  rclcpp::Time timestamp;
  if (use_gst_timestamps_) {
    timestamp = rclcpp::Time(GST_TIME_AS_NSECONDS(buf->pts + bt) + time_offset_);
  } else {
    timestamp = now();
  }

  
  // Get the image width and height
  GstPad * pad = gst_element_get_static_pad(sink_, "sink");
  const GstCaps * caps = gst_pad_get_current_caps(pad);
  GstStructure * structure = gst_caps_get_structure(caps, 0);
  gst_structure_get_int(structure, "width", &width_);
  gst_structure_get_int(structure, "height", &height_);
  gst_object_unref(pad);

  // Update header information
  sensor_msgs::msg::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
  cur_cinfo.header.stamp = timestamp;
  cur_cinfo.header.frame_id = frame_id_;

  if (image_encoding_ == "jpeg") {
    sensor_msgs::msg::CompressedImage img;
    img.header = cur_cinfo.header;
    img.format = "jpeg";
    img.data.resize(buf_size);
    std::copy(buf_data, buf_data + buf_size, img.data.begin());
    jpeg_pub_->publish(img);
    cinfo_pub_->publish(cur_cinfo);
  } else {
    const unsigned int expected_frame_size =
      width_ * height_ * sensor_msgs::image_encodings::numChannels(image_encoding_);
    if (buf_size < expected_frame_size) {
      RCLCPP_WARN_STREAM(get_logger(),
        "Buffer underflow: expected " << expected_frame_size << " bytes, got " << buf_size);
    }
    sensor_msgs::msg::Image img;
    img.header = cur_cinfo.header;
    img.width = width_;
    img.height = height_;
    img.encoding = image_encoding_;
    img.is_bigendian = false;
    img.step = width_ * sensor_msgs::image_encodings::numChannels(image_encoding_);
    img.data.resize(expected_frame_size);
    std::copy(buf_data,
              buf_data + std::min(buf_size, static_cast<gsize>(expected_frame_size)),
              img.data.begin());
    camera_pub_.publish(img, cur_cinfo);
  }

  // Release the buffer
  gst_memory_unmap(memory, &info);
  gst_memory_unref(memory);
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

void GSCam::cleanup_stream()
{
  RCLCPP_INFO(get_logger(), "Cleaning up GStreamer pipeline...");
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
}

void GSCam::gst_eos_cb(GstAppSink * appsink, gpointer user_data)
{
  GSCam * self = static_cast<GSCam *>(user_data);
  RCLCPP_INFO(self->get_logger(), "End-of-stream reached.");
  self->stop_signal_ = true;
}

GstFlowReturn GSCam::gst_new_preroll_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}

GstFlowReturn GSCam::gst_new_sample_cb(GstAppSink * appsink, gpointer user_data)
{
  GSCam * self = static_cast<GSCam *>(user_data);
  return self->on_new_sample(appsink);
}
}  // namespace gscam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam::GSCam)
