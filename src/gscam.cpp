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
  // gsconfig_(""),
  // pipeline_(NULL),
  // sink_(NULL),
  camera_info_manager_(this),
  stop_signal_(false)
{
  pipeline_thread_ = std::thread(
    [this]()
    {
      run();
    });
}

GSCam::~GSCam()
{
  stop_signal_ = true;
  pipeline_thread_.join();
}

bool GSCam::configure()
{
  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  bool gsconfig_rosparam_defined = false;
  char * gsconfig_env = NULL;

  std::map<std::string, std::string> gsconfig_rosparam;
  gsconfig_rosparam["left"] = declare_parameter("gscam_config_left", "");
  gsconfig_rosparam["middle"] = declare_parameter("gscam_config_middle", "");
  gsconfig_rosparam["right"] = declare_parameter("gscam_config_right", "");
  gsconfig_rosparam_defined = !(gsconfig_rosparam["left"].empty() || gsconfig_rosparam["middle"].empty() || gsconfig_rosparam["right"].empty());
  gsconfig_env = getenv("GSCAM_CONFIG");

  if (gsconfig_rosparam_defined == false) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Wrong gscam_config" << "\"");
      return false;
  }

    gsconfig_["left"] = gsconfig_rosparam["left"];
    gsconfig_["middle"] = gsconfig_rosparam["middle"];
    gsconfig_["right"] = gsconfig_rosparam["right"];
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Using gstreamer config from rosparam: \"" << gsconfig_rosparam["left"]
      << "\n" << gsconfig_rosparam["middle"]
      << "\n" << gsconfig_rosparam["right"] << "\"");
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
    RCLCPP_FATAL_STREAM(get_logger(), "Unsupported image encoding: " + image_encoding_);
  }

  camera_info_manager_.setCameraName(camera_name_);

  if (camera_info_manager_.validateURL(camera_info_url_)) {
    camera_info_manager_.loadCameraInfo(camera_info_url_);
    RCLCPP_INFO_STREAM(get_logger(), "Loaded camera calibration from " << camera_info_url_);
  } else {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Camera info at: " << camera_info_url_ << " not found. Using an uncalibrated config.");
  }

  // Get TF Frame
  frame_id_ = declare_parameter("frame_id", "camera_frame");
  if (frame_id_ == "camera_frame") {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "No camera frame_id set, using frame \"" << frame_id_ << "\".");
  }

  use_sensor_data_qos_ = declare_parameter("use_sensor_data_qos", false);

  return true;
}

bool GSCam::init_stream()
{
  if (!gst_is_initialized()) {
    // Initialize gstreamer pipeline
    RCLCPP_DEBUG_STREAM(get_logger(), "Initializing gstreamer...");
    gst_init(0, 0);
  }

  RCLCPP_DEBUG_STREAM(get_logger(), "Gstreamer Version: " << gst_version_string() );

  GError * error = 0;  // Assignment to zero is a gst requirement
  pipelines["left"] = gst_parse_launch(gsconfig_["left"].c_str(), &error);
  pipelines["middle"] = gst_parse_launch(gsconfig_["middle"].c_str(), &error);
  pipelines["right"] = gst_parse_launch(gsconfig_["right"].c_str(), &error);
  if (pipelines["left"] == NULL || pipelines["middle"] == NULL || pipelines["right"] == NULL) {
    RCLCPP_FATAL_STREAM(get_logger(), error->message);
    return false;
  }

  // Create RGB sink
  sinks["left"] = gst_element_factory_make("appsink", NULL);
  sinks["middle"] = gst_element_factory_make("appsink", NULL);
  sinks["right"] = gst_element_factory_make("appsink", NULL);
  std::map<std::string, GstCaps *> caps;
  caps["left"] = gst_app_sink_get_caps(GST_APP_SINK(sinks["left"]));
  caps["middle"] = gst_app_sink_get_caps(GST_APP_SINK(sinks["middle"]));
  caps["right"] = gst_app_sink_get_caps(GST_APP_SINK(sinks["right"]));


  // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps["left"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGB",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps["left"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "GRAY8",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps["left"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "UYVY",
      NULL);
  } else if (image_encoding_ == "jpeg") {
    caps["left"] = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }

  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps["middle"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGB",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps["middle"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "GRAY8",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps["middle"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "UYVY",
      NULL);
  } else if (image_encoding_ == "jpeg") {
    caps["middle"] = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }

  if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
    caps["right"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "RGB",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
    caps["right"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "GRAY8",
      NULL);
  } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
    caps["right"] = gst_caps_new_simple(
      "video/x-raw",
      "format", G_TYPE_STRING, "UYVY",
      NULL);
  } else if (image_encoding_ == "jpeg") {
    caps["right"] = gst_caps_new_simple("image/jpeg", NULL, NULL);
  }

  gst_app_sink_set_caps(GST_APP_SINK(sinks["left"]), caps["left"]);
  gst_caps_unref(caps["left"]);

  gst_app_sink_set_caps(GST_APP_SINK(sinks["middle"]), caps["middle"]);
  gst_caps_unref(caps["middle"]);

  gst_app_sink_set_caps(GST_APP_SINK(sinks["right"]), caps["right"]);
  gst_caps_unref(caps["right"]);

  // Set whether the sink should sync
  // Sometimes setting this to true can cause a large number of frames to be
  // dropped
  gst_base_sink_set_sync(
    GST_BASE_SINK(sinks["left"]),
    (sync_sink_) ? TRUE : FALSE);
  gst_base_sink_set_sync(
    GST_BASE_SINK(sinks["middle"]),
    (sync_sink_) ? TRUE : FALSE);
  gst_base_sink_set_sync(
    GST_BASE_SINK(sinks["right"]),
    (sync_sink_) ? TRUE : FALSE);

  
  if (GST_IS_PIPELINE(pipelines["left"]) && GST_IS_PIPELINE(pipelines["middle"]) && GST_IS_PIPELINE(pipelines["right"])) {
    // GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
    std::map<std::string, GstPad *> outpads;
    outpads["left"] = gst_bin_find_unlinked_pad(GST_BIN(pipelines["left"]), GST_PAD_SRC);
    outpads["middle"] = gst_bin_find_unlinked_pad(GST_BIN(pipelines["middle"]), GST_PAD_SRC);
    outpads["right"] = gst_bin_find_unlinked_pad(GST_BIN(pipelines["right"]), GST_PAD_SRC);
    g_assert(outpads["left"]);
    g_assert(outpads["middle"]);
    g_assert(outpads["right"]);

    std::map<std::string, GstElement *> outelements;
    outelements["left"] = gst_pad_get_parent_element(outpads["left"]);
    outelements["middle"] = gst_pad_get_parent_element(outpads["middle"]);
    outelements["right"] = gst_pad_get_parent_element(outpads["right"]);

    g_assert(outelements["left"]);
    g_assert(outelements["middle"]);
    g_assert(outelements["right"]);

    gst_object_unref(outpads["left"]);
    gst_object_unref(outpads["middle"]);
    gst_object_unref(outpads["right"]);

    if(!gst_bin_add(GST_BIN(pipelines["left"]), sinks["left"]))
    {
      RCLCPP_FATAL(get_logger(), "gst_bin_add_left() failed");
      gst_object_unref(outelements["left"]);
      gst_object_unref(pipelines["left"]);
      return false;
    }
    if(!gst_bin_add(GST_BIN(pipelines["middle"]), sinks["middle"]))
    {
      RCLCPP_FATAL(get_logger(), "gst_bin_add_middle() failed");
      gst_object_unref(outelements["middle"]);
      gst_object_unref(pipelines["middle"]);
      return false;
    }
    if(!gst_bin_add(GST_BIN(pipelines["right"]), sinks["right"]))
    {
      RCLCPP_FATAL(get_logger(), "gst_bin_add_right() failed");
      gst_object_unref(outelements["right"]);
      gst_object_unref(pipelines["right"]);
      return false;
    }

    if (!gst_element_link(outelements["left"], sinks["left"])) {
      RCLCPP_FATAL(
        get_logger(), "GStreamer: cannot link outelement left(\"%s\") -> sink\n",
        gst_element_get_name(outelements["left"]));
      gst_object_unref(outelements["left"]);
      gst_object_unref(pipelines["left"]);
      return false;
    }
    if (!gst_element_link(outelements["middle"], sinks["middle"])) {
      RCLCPP_FATAL(
        get_logger(), "GStreamer: cannot link outelement middle(\"%s\") -> sink\n",
        gst_element_get_name(outelements["middle"]));
      gst_object_unref(outelements["middle"]);
      gst_object_unref(pipelines["middle"]);
      return false;
    }
    if (!gst_element_link(outelements["right"], sinks["right"])) {
      RCLCPP_FATAL(
        get_logger(), "GStreamer: cannot link outelement right(\"%s\") -> sink\n",
        gst_element_get_name(outelements["right"]));
      gst_object_unref(outelements["right"]);
      gst_object_unref(pipelines["right"]);
      return false;
    }

    gst_object_unref(outelements["left"]);
    gst_object_unref(outelements["middle"]);
    gst_object_unref(outelements["right"]);
  } else {
    std::map<std::string, GstElement *> launchpipes;
    launchpipes["left"] = pipelines["left"];
    launchpipes["middle"] = pipelines["middle"];
    launchpipes["right"] = pipelines["right"];

    pipelines["left"] = gst_pipeline_new(NULL);
    pipelines["middle"] = gst_pipeline_new(NULL);
    pipelines["right"] = gst_pipeline_new(NULL);

    g_assert(pipelines["left"]);
    g_assert(pipelines["middle"]);
    g_assert(pipelines["right"]);

    gst_object_unparent(GST_OBJECT(launchpipes["left"]));
    gst_object_unparent(GST_OBJECT(launchpipes["middle"]));
    gst_object_unparent(GST_OBJECT(launchpipes["right"]));

    gst_bin_add_many(GST_BIN(pipelines["left"]), launchpipes["left"], sinks["left"], NULL);
    gst_bin_add_many(GST_BIN(pipelines["middle"]), launchpipes["middle"], sinks["middle"], NULL);
    gst_bin_add_many(GST_BIN(pipelines["right"]), launchpipes["right"], sinks["right"], NULL);

    if (!gst_element_link(launchpipes["left"], sinks["left"])) {
      RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe left -> sink");
      gst_object_unref(pipelines["left"]);
      return false;
    }
    if (!gst_element_link(launchpipes["middle"], sinks["middle"])) {
      RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe middle -> sink");
      gst_object_unref(pipelines["middle"]);
      return false;
    }
    if (!gst_element_link(launchpipes["right"], sinks["right"])) {
      RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe right -> sink");
      gst_object_unref(pipelines["right"]);
      return false;
    }
  }

  // Calibration between ros::Time and gst timestamps
  GstClock * clock = gst_system_clock_obtain();
  GstClockTime ct = gst_clock_get_time(clock);
  gst_object_unref(clock);
  time_offset_ = now().nanoseconds() - GST_TIME_AS_NSECONDS(ct);
  RCLCPP_INFO(get_logger(), "Time offset: %.6f", rclcpp::Time(time_offset_).seconds());

  gst_element_set_state(pipelines["left"], GST_STATE_PAUSED);
  gst_element_set_state(pipelines["middle"], GST_STATE_PAUSED);
  gst_element_set_state(pipelines["right"], GST_STATE_PAUSED);

  if (gst_element_get_state(pipelines["left"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
  }
  if (gst_element_get_state(pipelines["middle"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
  }
  if (gst_element_get_state(pipelines["right"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
    return false;
  } else {
    RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
  }

  // Create ROS camera interface
  const auto qos = use_sensor_data_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS{1};
  if (image_encoding_ == "jpeg") {
    jpeg_pub_ =
      create_publisher<sensor_msgs::msg::CompressedImage>(
      "image_raw/compressed", qos);
    cinfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", qos);
  } else {
    camera_pub_ = image_transport::create_camera_publisher(
      this, "image_raw", qos.get_rmw_qos_profile());
  }

  return true;
}

void GSCam::publish_stream()
{
  RCLCPP_INFO_STREAM(get_logger(), "Publishing stream...");

  // Pre-roll camera if needed
  if (preroll_) {
    RCLCPP_DEBUG(get_logger(), "Performing preroll...");

    // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
    // I am told this is needed and am erring on the side of caution.
    gst_element_set_state(pipelines["left"], GST_STATE_PLAYING);
    if (gst_element_get_state(pipelines["left"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PLAY left during preroll.");
      return;
    } else {
      RCLCPP_DEBUG(get_logger(), "Left stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipelines["middle"], GST_STATE_PLAYING);
    if (gst_element_get_state(pipelines["middle"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PLAY middle during preroll.");
      return;
    } else {
      RCLCPP_DEBUG(get_logger(), "Middle stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipelines["right"], GST_STATE_PLAYING);
    if (gst_element_get_state(pipelines["right"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PLAY right during preroll.");
      return;
    } else {
      RCLCPP_DEBUG(get_logger(), "Right stream is PLAYING in preroll.");
    }

    gst_element_set_state(pipelines["left"], GST_STATE_PAUSED);
    if (gst_element_get_state(pipelines["left"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PAUSE left.");
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Left stream is PAUSED in preroll.");
    }

    gst_element_set_state(pipelines["middle"], GST_STATE_PAUSED);
    if (gst_element_get_state(pipelines["middle"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PAUSE middle.");
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Middle stream is PAUSED in preroll.");
    }

    gst_element_set_state(pipelines["right"], GST_STATE_PAUSED);
    if (gst_element_get_state(pipelines["right"], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to PAUSE right.");
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Right stream is PAUSED in preroll.");
    }
  }

  if (gst_element_set_state(pipelines["left"], GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Could not start left stream!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Started left stream.");

  if (gst_element_set_state(pipelines["middle"], GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Could not start middle stream!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Started middle stream.");

  if (gst_element_set_state(pipelines["right"], GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Could not start right stream!");
    return;
  }
  RCLCPP_INFO(get_logger(), "Started right stream.");
  
  std::map<std::string, GstSample *> samples;
  std::map<std::string, GstBuffer *> bufs;
  std::map<std::string, GstMemory *> memories;
  std::map<std::string, GstMapInfo> infos;
  std::map<std::string, gsize> buf_sizes;
  std::map<std::string, guint8 *> buf_datas;
  std::map<std::string, GstClockTime> bts;
  std::map<std::string, GstPad *> pads;
  std::map<std::string, GstCaps *> caps;
  std::map<std::string, GstStructure *> structures;
  // Poll the data as fast a spossible
  while (!stop_signal_ && rclcpp::ok()) {
    // This should block until a new frame is awake, this way, we'll run at the
    // actual capture framerate of the device.
    // RCLCPP_DEBUG(get_logger(), "Getting data...");
    
    samples["left"] = gst_app_sink_pull_sample(GST_APP_SINK(sinks["left"]));
    samples["middle"] = gst_app_sink_pull_sample(GST_APP_SINK(sinks["middle"]));
    samples["right"] = gst_app_sink_pull_sample(GST_APP_SINK(sinks["right"]));
    if (!samples["left"]) {
      RCLCPP_ERROR(get_logger(), "Could not get left gstreamer sample.");
      break;
    }
    if (!samples["middle"]) {
      RCLCPP_ERROR(get_logger(), "Could not get middle gstreamer sample.");
      break;
    }
    if (!samples["right"]) {
      RCLCPP_ERROR(get_logger(), "Could not get right gstreamer sample.");
      break;
    }
    bufs["left"] = gst_sample_get_buffer(samples["left"]);
    bufs["middle"] = gst_sample_get_buffer(samples["middle"]);
    bufs["right"] = gst_sample_get_buffer(samples["right"]);
    memories["left"] = gst_buffer_get_memory(bufs["left"], 0);
    memories["middle"] = gst_buffer_get_memory(bufs["middle"], 0);
    memories["right"] = gst_buffer_get_memory(bufs["right"], 0);

    if(!gst_memory_map(memories["left"], &infos["left"], GST_MAP_READ))
    {
      RCLCPP_ERROR(get_logger(), "Could not map left memory.");
      break;
    }
    if(!gst_memory_map(memories["middle"], &infos["middle"], GST_MAP_READ))
    {
      RCLCPP_ERROR(get_logger(), "Could not map middle memory.");
      break;
    }
    if(!gst_memory_map(memories["right"], &infos["right"], GST_MAP_READ))
    {
      RCLCPP_ERROR(get_logger(), "Could not map right memory.");
      break;
    }
    buf_sizes["left"] = infos["left"].size;
    buf_sizes["middle"] = infos["middle"].size;
    buf_sizes["right"] = infos["right"].size;
    buf_datas["left"] = infos["left"].data;
    buf_datas["middle"] = infos["middle"].data;
    buf_datas["right"] = infos["right"].data;

    bts["left"] = gst_element_get_base_time(pipelines["left"]);
    bts["middle"] = gst_element_get_base_time(pipelines["middle"]);
    bts["right"] = gst_element_get_base_time(pipelines["right"]);
    // RCLCPP_INFO(
    //   get_logger(),
    //   "New buffer: timestamp %.6f %lu %lu %.3f",
    //   GST_TIME_AS_USECONDS(buf->timestamp + bt) / 1e6 + time_offset_,
    //   buf->timestamp, bt, time_offset_);


#if 0
    GstFormat fmt = GST_FORMAT_TIME;
    gint64 current = -1;

    Query the current position of the stream
    if (gst_element_query_position(pipeline_, &fmt, &current)) {
      RCLCPP_INFO_STREAM(get_logger(), "Position " << current);
    }
#endif

    // Stop on end of stream
    if (!bufs["left"]) {
      RCLCPP_INFO(get_logger(), "Left stream ended.");
      break;
    }
    if (!bufs["middle"]) {
      RCLCPP_INFO(get_logger(), "Middle stream ended.");
      break;
    }
    if (!bufs["right"]) {
      RCLCPP_INFO(get_logger(), "Right stream ended.");
      break;
    }

    // RCLCPP_DEBUG(get_logger(), "Got data.");

    // Get the image width and height
    pads["left"] = gst_element_get_static_pad(sinks["left"], "sink");
    pads["middle"] = gst_element_get_static_pad(sinks["middle"], "sink");
    pads["right"] = gst_element_get_static_pad(sinks["right"], "sink");
    caps["left"] = gst_pad_get_current_caps(pads["left"]);
    caps["middle"] = gst_pad_get_current_caps(pads["middle"]);
    caps["right"] = gst_pad_get_current_caps(pads["right"]);

    structures["left"] = gst_caps_get_structure(caps["left"], 0);
    structures["middle"] = gst_caps_get_structure(caps["middle"], 0);
    structures["right"] = gst_caps_get_structure(caps["right"], 0);

    gst_structure_get_int(structures["left"], "width", &widths["left"]);
    gst_structure_get_int(structures["left"], "height", &heights["left"]);
    gst_structure_get_int(structures["middle"], "width", &widths["middle"]);
    gst_structure_get_int(structures["middle"], "height", &heights["middle"]);
    gst_structure_get_int(structures["right"], "width", &widths["right"]);
    gst_structure_get_int(structures["right"], "height", &heights["right"]);

    // Update header information
    sensor_msgs::msg::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo;
    cinfo.reset(new sensor_msgs::msg::CameraInfo(cur_cinfo));
    if (use_gst_timestamps_) {
      cinfo->header.stamp = rclcpp::Time(GST_TIME_AS_NSECONDS(bufs["left"]->pts + bts["left"]) + time_offset_); // set same timestamp for all streams
    } else {
      cinfo->header.stamp = now();
    }
    // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
    cinfo->header.frame_id = frame_id_;
    for(const auto& camera_name : camera_names)
    {
      if (image_encoding_ == "jpeg") {
        sensor_msgs::msg::CompressedImage::SharedPtr img(new sensor_msgs::msg::CompressedImage());
        img->header = cinfo->header;
        img->format = "jpeg";
        img->data.resize(buf_sizes[camera_name]);
        std::copy(
          buf_datas[camera_name], (buf_datas[camera_name]) + (buf_sizes[camera_name]),
          img->data.begin());
        jpeg_pub_->publish(*img);
        cinfo_pub_->publish(*cinfo);
      } else {
        // Complain if the returned buffer is smaller than we expect
        const unsigned int expected_frame_size =
          widths[camera_name] * heights[camera_name] * sensor_msgs::image_encodings::numChannels(image_encoding_);

        if (buf_sizes[camera_name] < expected_frame_size) {
          RCLCPP_WARN_STREAM(
            get_logger(), "GStreamer image buffer underflow: Expected frame to be " <<
              expected_frame_size << " bytes but got only " <<
              buf_sizes[camera_name] << " bytes. (make sure frames are correctly encoded)");
        }

        // Construct Image message
        sensor_msgs::msg::Image::SharedPtr img(new sensor_msgs::msg::Image());

        img->header = cinfo->header;

        // Image data and metadata
        img->width = widths[camera_name];
        img->height = heights[camera_name];
        img->encoding = image_encoding_;
        img->is_bigendian = false;
        img->data.resize(expected_frame_size);

        // Copy only the data we received
        // Since we're publishing shared pointers, we need to copy the image so
        // we can free the buffer allocated by gstreamer
        img->step = widths[camera_name] * sensor_msgs::image_encodings::numChannels(image_encoding_);

        std::copy(
          buf_datas[camera_name],
          (buf_datas[camera_name]) + (buf_sizes[camera_name]),
          img->data.begin());

        // Publish the image/info
        camera_pub_.publish(img, cinfo);
      }

    // Release the buffer
      if (bufs[camera_name]) {
        gst_memory_unmap(memories[camera_name], &infos[camera_name]);
        gst_memory_unref(memories[camera_name]);
        gst_sample_unref(samples[camera_name]);
      }
    }
  }
}

void GSCam::cleanup_stream()
{
  // Clean up
  RCLCPP_INFO(get_logger(), "Stopping gstreamer pipeline...");

    for(const auto& camera_name : camera_names)
    {
      if (pipelines[camera_name]) {
        gst_element_set_state(pipelines[camera_name], GST_STATE_NULL);
        gst_object_unref(pipelines[camera_name]);
        pipelines[camera_name] = nullptr;
      }
    }
}

void GSCam::run()
{
  if (!this->configure()) {
    RCLCPP_FATAL(get_logger(), "Failed to configure gscam!");
    return;
  }

  while (!stop_signal_ && rclcpp::ok()) {
    if (!this->init_stream()) {
      RCLCPP_FATAL(get_logger(), "Failed to initialize gscam stream!");
      break;
    }

    // Block while publishing
    this->publish_stream();

    this->cleanup_stream();

    RCLCPP_INFO(get_logger(), "GStreamer stream stopped!");

    if (reopen_on_eof_) {
      RCLCPP_INFO(get_logger(), "Reopening stream...");
    } else {
      RCLCPP_INFO(get_logger(), "Cleaning up stream and exiting...");
      break;
    }
  }
  rclcpp::shutdown();
}

// Example callbacks for appsink
// TODO(someone): enable callback-based capture
void gst_eos_cb(GstAppSink * appsink, gpointer user_data)
{
}
GstFlowReturn gst_new_preroll_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}
GstFlowReturn gst_new_asample_cb(GstAppSink * appsink, gpointer user_data)
{
  return GST_FLOW_OK;
}

}  // namespace gscam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(gscam::GSCam)
