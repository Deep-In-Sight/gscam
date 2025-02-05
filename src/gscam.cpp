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

std::vector<std::string> splitString(const std::string &s, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(s);
    std::string token;
    while (std::getline(ss, token, delimiter))
    {
        if (!token.empty())
        {
            tokens.push_back(token);
        }
    }
    return tokens;
}

bool GSCam::configure()
{
  // Get gstreamer configuration
  // (either from environment variable or ROS param)
  char * gsconfig_env = NULL;

  // Get TF Frame
  std::string frame_id_list_str = declare_parameter("frame_id_list", "");
  if(frame_id_list_str == "")
  {
    RCLCPP_ERROR(get_logger(),"No camera frame_id set.");
  }
  frame_id_lists = splitString(frame_id_list_str, ' ');  

  RCLCPP_INFO(this->get_logger(), "Loaded camera list:");

  std::map<std::string, std::string> gsconfig_rosparam;
  for (const auto &camera : frame_id_lists)
  {
      RCLCPP_INFO(this->get_logger(), "- %s", camera.c_str());
      std::string camera_param_name = "gscam_config_" + camera;
      gsconfig_rosparam[camera] = declare_parameter(camera_param_name, "");
      gsconfig_env = getenv("GSCAM_CONFIG");

      if (gsconfig_rosparam[camera].empty()) {
        RCLCPP_INFO_STREAM(
          get_logger(),
          "Wrong gscam_config" << "\"");
          return false;
      }

      gsconfig_[camera] = gsconfig_rosparam[camera];
      RCLCPP_INFO_STREAM(
        get_logger(),
        "Using gstreamer config from rosparam: \n" << gsconfig_rosparam[camera] << "\n");
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

  std::map<std::string, GstCaps *> caps;
  std::map<std::string, GstPad *> outpads;
  std::map<std::string, GstElement *> outelements;
  std::map<std::string, GstElement *> launchpipes;
  for (const auto &camera : frame_id_lists)
  {
      pipelines[camera] = gst_parse_launch(gsconfig_[camera].c_str(), &error);
      if (pipelines[camera] == NULL) {
        RCLCPP_FATAL_STREAM(get_logger(), error->message);
        return false;
      }

      // Create RGB sink
      sinks[camera] = gst_element_factory_make("appsink", NULL);
      
      caps[camera] = gst_app_sink_get_caps(GST_APP_SINK(sinks[camera]));


      // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
      if (image_encoding_ == sensor_msgs::image_encodings::RGB8) {
        caps[camera] = gst_caps_new_simple(
          "video/x-raw",
          "format", G_TYPE_STRING, "RGB",
          NULL);
      } else if (image_encoding_ == sensor_msgs::image_encodings::MONO8) {
        caps[camera] = gst_caps_new_simple(
          "video/x-raw",
          "format", G_TYPE_STRING, "GRAY8",
          NULL);
      } else if (image_encoding_ == sensor_msgs::image_encodings::YUV422) {
        caps[camera] = gst_caps_new_simple(
          "video/x-raw",
          "format", G_TYPE_STRING, "UYVY",
          NULL);
      } else if (image_encoding_ == "jpeg") {
        caps[camera] = gst_caps_new_simple("image/jpeg", NULL, NULL);
      }

      gst_app_sink_set_caps(GST_APP_SINK(sinks[camera]), caps[camera]);
      gst_caps_unref(caps[camera]);

      // Set whether the sink should sync
      // Sometimes setting this to true can cause a large number of frames to be
      // dropped
      gst_base_sink_set_sync(
        GST_BASE_SINK(sinks[camera]),
        (sync_sink_) ? TRUE : FALSE);
      
      if (GST_IS_PIPELINE(pipelines[camera])) {
        // GstPad * outpad = gst_bin_find_unlinked_pad(GST_BIN(pipeline_), GST_PAD_SRC);
        outpads[camera] = gst_bin_find_unlinked_pad(GST_BIN(pipelines[camera]), GST_PAD_SRC);
        g_assert(outpads[camera]);
        
        outelements[camera] = gst_pad_get_parent_element(outpads[camera]);

        g_assert(outelements[camera]);
        
        gst_object_unref(outpads[camera]);

        if(!gst_bin_add(GST_BIN(pipelines[camera]), sinks[camera]))
        {
          RCLCPP_FATAL(get_logger(), "gst_bin_add_%s() failed", camera.c_str());
          gst_object_unref(outelements[camera]);
          gst_object_unref(pipelines[camera]);
          return false;
        }

        if (!gst_element_link(outelements[camera], sinks[camera])) {
          RCLCPP_FATAL(
              get_logger(), "GStreamer: cannot link outelement %s(\"%s\") -> sink\n",
              camera.c_str(), gst_element_get_name(outelements[camera])
          );

          gst_object_unref(outelements[camera]);
          gst_object_unref(pipelines[camera]);
          return false;
        }

        gst_object_unref(outelements[camera]);
      } else {
        launchpipes[camera] = pipelines[camera];
        
        pipelines[camera] = gst_pipeline_new(NULL);

        g_assert(pipelines[camera]);

        gst_object_unparent(GST_OBJECT(launchpipes[camera]));

        gst_bin_add_many(GST_BIN(pipelines[camera]), launchpipes[camera], sinks[camera], NULL);

        if (!gst_element_link(launchpipes[camera], sinks[camera])) {
          RCLCPP_FATAL(get_logger(), "GStreamer: cannot link launchpipe %s -> sink", camera.c_str());
          gst_object_unref(pipelines[camera]);
          return false;
        }
      }

      // Calibration between ros::Time and gst timestamps
      GstClock * clock = gst_system_clock_obtain();
      GstClockTime ct = gst_clock_get_time(clock);
      gst_object_unref(clock);
      time_offset_ = now().nanoseconds() - GST_TIME_AS_NSECONDS(ct);
      RCLCPP_INFO(get_logger(), "Time offset: %.6f", rclcpp::Time(time_offset_).seconds());

      gst_element_set_state(pipelines[camera], GST_STATE_PAUSED);

      if (gst_element_get_state(pipelines[camera], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_FATAL(get_logger(), "Failed to PAUSE stream, check your gstreamer configuration.");
        return false;
      } else {
        RCLCPP_DEBUG_STREAM(get_logger(), "Stream is PAUSED.");
      }
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


    for (const auto &camera : frame_id_lists)
    {
      // The PAUSE, PLAY, PAUSE, PLAY cycle is to ensure proper pre-roll
      // I am told this is needed and am erring on the side of caution.
      gst_element_set_state(pipelines[camera], GST_STATE_PLAYING);
      if (gst_element_get_state(pipelines[camera], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(get_logger(), "Failed to PLAY %s during preroll.", camera.c_str()); 
        return;
      } else {
        RCLCPP_DEBUG(get_logger(), "%s stream is PLAYING in preroll.", camera.c_str());
      }

      gst_element_set_state(pipelines[camera], GST_STATE_PAUSED);
      if (gst_element_get_state(pipelines[camera], NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
        RCLCPP_ERROR(get_logger(), "Failed to PAUSE %s.", camera.c_str());
        return;
      } else {
        RCLCPP_INFO(get_logger(), "%s stream is PAUSED in preroll.", camera.c_str());
      }
    }
  }

  for (const auto &camera : frame_id_lists)
  {
    if (gst_element_set_state(pipelines[camera], GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Could not start %s stream!", camera.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Started %s stream.", camera.c_str());
  }

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
    
    for (const auto &camera : frame_id_lists)
    {
      samples[camera] = gst_app_sink_pull_sample(GST_APP_SINK(sinks[camera]));
      if (!samples[camera]) {
        RCLCPP_ERROR(get_logger(), "Could not get %s gstreamer sample.", camera.c_str());
        return;
      }
    }


    for (const auto &camera : frame_id_lists)
    {
      bufs[camera] = gst_sample_get_buffer(samples[camera]);
      memories[camera] = gst_buffer_get_memory(bufs[camera], 0);

      if(!gst_memory_map(memories[camera], &infos[camera], GST_MAP_READ))
      {
        RCLCPP_ERROR(get_logger(), "Could not map %s memory.", camera.c_str());
        return;
      }
      buf_sizes[camera] = infos[camera].size;
      buf_datas[camera] = infos[camera].data;

      bts[camera] = gst_element_get_base_time(pipelines[camera]);
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
      if (!bufs[camera]) {
        RCLCPP_INFO(get_logger(), "%s stream ended.", camera.c_str());
        return;
      }

      // RCLCPP_DEBUG(get_logger(), "Got data.");

      // Get the image width and height
      pads[camera] = gst_element_get_static_pad(sinks[camera], "sink");
      caps[camera] = gst_pad_get_current_caps(pads[camera]);

      structures[camera] = gst_caps_get_structure(caps[camera], 0);

      gst_structure_get_int(structures[camera], "width", &widths[camera]);
      gst_structure_get_int(structures[camera], "height", &heights[camera]);
    }
    // Update header information
    sensor_msgs::msg::CameraInfo cur_cinfo = camera_info_manager_.getCameraInfo();
    sensor_msgs::msg::CameraInfo::SharedPtr cinfo;
    cinfo.reset(new sensor_msgs::msg::CameraInfo(cur_cinfo));
    if (use_gst_timestamps_) {
      cinfo->header.stamp = rclcpp::Time(GST_TIME_AS_NSECONDS(bufs[frame_id_lists[0]]->pts + bts[frame_id_lists[0]]) + time_offset_); // set same timestamp for all streams
    } else {
      cinfo->header.stamp = now();
    }

    // RCLCPP_INFO(get_logger(), "Image time stamp: %.3f",cinfo->header.stamp.toSec());
    for(const auto& camera_name : frame_id_lists)
    {
      cinfo->header.frame_id = camera_name;
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

    for(const auto& camera_name : frame_id_lists)
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
