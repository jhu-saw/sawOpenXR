#include <sawOpenXR/OpenXRRuntime.h>

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <vulkan/vulkan.h>
#define XR_USE_GRAPHICS_API_VULKAN
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr auto kVideoReconnectDelay = std::chrono::seconds(5);

void xr_check(XrResult result, const char *expression) {
  if (XR_FAILED(result)) {
    throw std::runtime_error(std::string(expression) +
                             " failed with OpenXR error " +
                             std::to_string(result));
  }
}

#define XR_CHECK(expr) xr_check((expr), #expr)

void vk_check(VkResult result, const char *expression) {
  if (result != VK_SUCCESS) {
    throw std::runtime_error(std::string(expression) +
                             " failed with Vulkan error " +
                             std::to_string(result));
  }
}

#define VK_CHECK(expr) vk_check((expr), #expr)

struct Mat4 {
  // Column-major, as consumed directly by GLSL mat4.
  float value[16]{};

  float &at(uint32_t row, uint32_t column) { return value[column * 4 + row]; }
  float at(uint32_t row, uint32_t column) const {
    return value[column * 4 + row];
  }
};

Mat4 multiply(const Mat4 &left, const Mat4 &right) {
  Mat4 result{};
  for (uint32_t row = 0; row < 4; ++row) {
    for (uint32_t column = 0; column < 4; ++column) {
      for (uint32_t item = 0; item < 4; ++item) {
        result.at(row, column) += left.at(row, item) * right.at(item, column);
      }
    }
  }
  return result;
}

Mat4 pose_matrix(const XrPosef &pose) {
  const auto &q = pose.orientation;
  const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
  const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
  const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
  Mat4 result{};
  result.at(0, 0) = 1.0f - 2.0f * (yy + zz);
  result.at(0, 1) = 2.0f * (xy - wz);
  result.at(0, 2) = 2.0f * (xz + wy);
  result.at(1, 0) = 2.0f * (xy + wz);
  result.at(1, 1) = 1.0f - 2.0f * (xx + zz);
  result.at(1, 2) = 2.0f * (yz - wx);
  result.at(2, 0) = 2.0f * (xz - wy);
  result.at(2, 1) = 2.0f * (yz + wx);
  result.at(2, 2) = 1.0f - 2.0f * (xx + yy);
  result.at(0, 3) = pose.position.x;
  result.at(1, 3) = pose.position.y;
  result.at(2, 3) = pose.position.z;
  result.at(3, 3) = 1.0f;
  return result;
}

Mat4 inverse_rigid_matrix(const XrPosef &pose) {
  const Mat4 transform = pose_matrix(pose);
  Mat4 result{};
  for (uint32_t row = 0; row < 3; ++row) {
    for (uint32_t column = 0; column < 3; ++column) {
      result.at(row, column) = transform.at(column, row);
    }
  }
  for (uint32_t row = 0; row < 3; ++row) {
    result.at(row, 3) = -(result.at(row, 0) * pose.position.x +
                          result.at(row, 1) * pose.position.y +
                          result.at(row, 2) * pose.position.z);
  }
  result.at(3, 3) = 1.0f;
  return result;
}

Mat4 projection_matrix(const XrFovf &fov) {
  const float left = std::tan(fov.angleLeft);
  const float right = std::tan(fov.angleRight);
  const float down = std::tan(fov.angleDown);
  const float up = std::tan(fov.angleUp);
  constexpr float near_plane = 0.05f;
  constexpr float far_plane = 100.0f;
  Mat4 result{};
  result.at(0, 0) = 2.0f / (right - left);
  // Vulkan has a positive-down clip-space Y axis and a 0..1 depth range.
  // This follows Khronos' XrMatrix4x4f_CreateProjection(..., GRAPHICS_VULKAN).
  result.at(1, 1) = 2.0f / (down - up);
  result.at(0, 2) = (right + left) / (right - left);
  result.at(1, 2) = (up + down) / (down - up);
  result.at(2, 2) = -far_plane / (far_plane - near_plane);
  result.at(2, 3) = -(far_plane * near_plane) / (far_plane - near_plane);
  result.at(3, 2) = -1.0f;
  return result;
}

XrQuaternionf quaternion_conjugate(const XrQuaternionf &q) {
  return {-q.x, -q.y, -q.z, q.w};
}

XrQuaternionf quaternion_multiply(const XrQuaternionf &left,
                                  const XrQuaternionf &right) {
  return {
      left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
      left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
      left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w,
      left.w * right.w - left.x * right.x - left.y * right.y -
          left.z * right.z};
}

XrVector3f rotate_vector(const XrQuaternionf &orientation,
                         const XrVector3f &vector) {
  const XrQuaternionf pure{vector.x, vector.y, vector.z, 0.0f};
  const XrQuaternionf rotated =
      quaternion_multiply(quaternion_multiply(orientation, pure),
                          quaternion_conjugate(orientation));
  return {rotated.x, rotated.y, rotated.z};
}

struct VulkanContext {
  VkInstance instance = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  VkQueue queue = VK_NULL_HANDLE;
  uint32_t queue_family = 0;
  VkCommandPool command_pool = VK_NULL_HANDLE;
  VkCommandBuffer command_buffer = VK_NULL_HANDLE;
  VkFence fence = VK_NULL_HANDLE;

  ~VulkanContext() {
    if (device != VK_NULL_HANDLE) {
      vkDeviceWaitIdle(device);
      if (fence != VK_NULL_HANDLE) {
        vkDestroyFence(device, fence, nullptr);
      }
      if (command_pool != VK_NULL_HANDLE) {
        vkDestroyCommandPool(device, command_pool, nullptr);
      }
      vkDestroyDevice(device, nullptr);
    }
    if (instance != VK_NULL_HANDLE) {
      vkDestroyInstance(instance, nullptr);
    }
  }
};

class App {
public:
  App(const std::string &video_pipeline, sawOpenXR::VideoType video_type,
      sawOpenXR::OpenXRRuntime::ControllerCallback controller_callback,
      sawOpenXR::OpenXRRuntime::GStreamerCallback gstreamer_status_callback,
      sawOpenXR::OpenXRRuntime::GStreamerCallback gstreamer_warning_callback,
      std::atomic_bool &stop_requested)
      : video_pipeline_(video_pipeline),
        video_type_(video_type),
        controller_callback_(std::move(controller_callback)),
        gstreamer_status_callback_(std::move(gstreamer_status_callback)),
        gstreamer_warning_callback_(std::move(gstreamer_warning_callback)),
        stop_requested_(stop_requested) {}

  void run() {
    gst_init(nullptr, nullptr);
    wait_for_initial_video_frame();
    if (stop_requested_.load()) {
      return;
    }
    create_instance();
    create_session();
    std::cout << "OpenXR session created. Put on the headset; Ctrl+C stops the "
                 "program.\n";
    render_loop();
  }

  ~App() {
    if (vk_.device != VK_NULL_HANDLE) {
      vkDeviceWaitIdle(vk_.device);
    }
    for (auto &eye : eye_swapchains_) {
      for (const auto framebuffer : eye.framebuffers) {
        if (framebuffer != VK_NULL_HANDLE) {
          vkDestroyFramebuffer(vk_.device, framebuffer, nullptr);
        }
      }
      for (const auto view : eye.image_views) {
        if (view != VK_NULL_HANDLE) {
          vkDestroyImageView(vk_.device, view, nullptr);
        }
      }
      if (eye.handle != XR_NULL_HANDLE) {
        xrDestroySwapchain(eye.handle);
      }
    }
    destroy_video_resources();
    if (render_pass_ != VK_NULL_HANDLE) {
      vkDestroyRenderPass(vk_.device, render_pass_, nullptr);
    }
    if (local_space_ != XR_NULL_HANDLE) {
      xrDestroySpace(local_space_);
    }
    for (const auto grip_space : grip_spaces_) {
      if (grip_space != XR_NULL_HANDLE) {
        xrDestroySpace(grip_space);
      }
    }
    if (session_ != XR_NULL_HANDLE) {
      xrDestroySession(session_);
    }
    if (action_set_ != XR_NULL_HANDLE) {
      xrDestroyActionSet(action_set_);
    }
    if (instance_ != XR_NULL_HANDLE) {
      xrDestroyInstance(instance_);
    }
    stop_test_source();
  }

private:
  void DispatchGStreamerStatus(const std::string &message) {
    if (gstreamer_status_callback_) {
      gstreamer_status_callback_(message);
    }
  }

  void DispatchGStreamerWarning(const std::string &message) {
    if (gstreamer_warning_callback_) {
      gstreamer_warning_callback_(message);
    }
  }

  XrInstance instance_ = XR_NULL_HANDLE;
  XrSystemId system_id_ = XR_NULL_SYSTEM_ID;
  XrSession session_ = XR_NULL_HANDLE;
  XrSpace local_space_ = XR_NULL_HANDLE;
  struct EyeSwapchain {
    XrSwapchain handle = XR_NULL_HANDLE;
    std::vector<VkImage> images;
    std::vector<VkImageView> image_views;
    std::vector<VkFramebuffer> framebuffers;
  };
  bool session_running_ = false;
  bool exit_requested_ = false;
  bool shutdown_exit_requested_ = false;
  std::chrono::steady_clock::time_point shutdown_deadline_{};
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  int64_t color_format_ = 0;
  XrEnvironmentBlendMode environment_blend_mode_ =
      XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
  std::vector<XrViewConfigurationView> config_views_;
  std::vector<XrView> views_;
  std::vector<EyeSwapchain> eye_swapchains_;
  VkRenderPass render_pass_ = VK_NULL_HANDLE;
  XrActionSet action_set_ = XR_NULL_HANDLE;
  XrAction quit_action_ = XR_NULL_HANDLE;
  XrAction grab_action_ = XR_NULL_HANDLE;
  XrAction reset_window_action_ = XR_NULL_HANDLE;
  XrAction thumbstick_action_ = XR_NULL_HANDLE;
  XrAction thumbstick_click_action_ = XR_NULL_HANDLE;
  XrAction front_trigger_action_ = XR_NULL_HANDLE;
  XrAction grip_pose_action_ = XR_NULL_HANDLE;
  std::array<XrPath, 2> hand_paths_{};
  std::array<XrSpace, 2> grip_spaces_{};
  bool quit_was_pressed_ = false;
  bool reset_window_was_pressed_ = false;
  std::array<bool, 2> grab_was_pressed_{};
  int grabbed_hand_ = -1;
  XrPosef video_window_pose_{{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.5f}};
  bool video_window_initialized_ = false;
  XrQuaternionf grab_start_orientation_{0.0f, 0.0f, 0.0f, 1.0f};
  XrQuaternionf grab_start_window_orientation_{0.0f, 0.0f, 0.0f, 1.0f};
  XrVector3f grab_start_position_{};
  XrVector3f grab_start_window_position_{};
  VulkanContext vk_;

  static constexpr uint32_t eye_count_ = 2;
  uint32_t eye_video_width_ = 0;
  uint32_t eye_video_height_ = 0;
  uint32_t source_video_width_ = 0;
  GstElement *test_pipeline_ = nullptr;
  GstAppSink *test_sink_ = nullptr;
  GstElement *video_queue_ = nullptr;
  GstVideoInfo test_video_info_{};
  uint64_t test_frame_count_ = 0;
  double latest_video_age_ms_ = 0.0;
  bool latest_video_age_valid_ = false;
  double sender_video_age_ms_ = 0.0;
  bool sender_video_age_valid_ = false;
  bool sender_timestamp_logged_ = false;
  bool has_test_frame_ = false;
  bool video_format_checked_ = false;
  std::chrono::steady_clock::time_point next_video_retry_{};
  struct VideoTexture {
    bool image_initialized = false;
    VkBuffer staging_buffer = VK_NULL_HANDLE;
    VkDeviceMemory staging_memory = VK_NULL_HANDLE;
    void *staging_mapped = nullptr;
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory image_memory = VK_NULL_HANDLE;
    VkImageView image_view = VK_NULL_HANDLE;
    VkDescriptorSet descriptor_set = VK_NULL_HANDLE;
  };
  std::array<VideoTexture, eye_count_> video_textures_{};
  VkSampler test_sampler_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout test_descriptor_set_layout_ = VK_NULL_HANDLE;
  VkDescriptorPool test_descriptor_pool_ = VK_NULL_HANDLE;
  VkPipelineLayout test_pipeline_layout_ = VK_NULL_HANDLE;
  VkPipeline test_graphics_pipeline_ = VK_NULL_HANDLE;

  PFN_xrGetVulkanGraphicsRequirements2KHR get_graphics_requirements_ = nullptr;
  PFN_xrCreateVulkanInstanceKHR create_vulkan_instance_ = nullptr;
  PFN_xrGetVulkanGraphicsDevice2KHR get_graphics_device_ = nullptr;
  PFN_xrCreateVulkanDeviceKHR create_vulkan_device_ = nullptr;
  std::string video_pipeline_;
  sawOpenXR::VideoType video_type_;
  sawOpenXR::OpenXRRuntime::ControllerCallback controller_callback_;
  sawOpenXR::OpenXRRuntime::GStreamerCallback gstreamer_status_callback_;
  sawOpenXR::OpenXRRuntime::GStreamerCallback gstreamer_warning_callback_;
  std::atomic_bool &stop_requested_;

  bool start_test_source() {
    const std::string pipeline_description =
        video_pipeline_ +
        " ! videoconvert ! "
        "video/x-raw,format=RGBA,pixel-aspect-ratio=1/1 ! "
        "appsink name=quest_test_sink max-buffers=1 drop=true sync=false "
        "processing-deadline=0 enable-last-sample=false";

    GError *error = nullptr;
    test_pipeline_ = gst_parse_launch(pipeline_description.c_str(), &error);
    if (test_pipeline_ == nullptr) {
      const std::string message = error != nullptr
                                      ? error->message
                                      : "unknown GStreamer pipeline error";
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("Could not create GStreamer video source: " +
                               message);
    }
    GstElement *sink_element =
        gst_bin_get_by_name(GST_BIN(test_pipeline_), "quest_test_sink");
    if (sink_element == nullptr) {
      throw std::runtime_error(
          "Could not find GStreamer appsink in video source");
    }
    test_sink_ = GST_APP_SINK(sink_element);
    gst_app_sink_set_drop(test_sink_, true);
    gst_app_sink_set_max_buffers(test_sink_, 1);

    video_queue_ =
        gst_bin_get_by_name(GST_BIN(test_pipeline_), "quest_video_queue");

    const GstStateChangeReturn state =
        gst_element_set_state(test_pipeline_, GST_STATE_PLAYING);
    if (state == GST_STATE_CHANGE_FAILURE) {
      DispatchGStreamerWarning(
          "Could not start the configured GStreamer video source; retrying.");
      stop_test_source();
      return false;
    }
    DispatchGStreamerStatus("Connecting GStreamer source: " + video_pipeline_);
    return true;
  }

  void stop_test_source() {
    if (test_pipeline_ != nullptr) {
      gst_element_set_state(test_pipeline_, GST_STATE_NULL);
    }
    if (test_sink_ != nullptr) {
      gst_object_unref(test_sink_);
      test_sink_ = nullptr;
    }

    if (video_queue_ != nullptr) {
      gst_object_unref(video_queue_);
      video_queue_ = nullptr;
    }

    if (test_pipeline_ != nullptr) {
      gst_object_unref(test_pipeline_);
      test_pipeline_ = nullptr;
    }
  }

  bool check_stream_bus() {
    if (test_pipeline_ == nullptr) {
      return false;
    }
    GstBus *bus = gst_element_get_bus(test_pipeline_);
    GstMessage *message = gst_bus_pop_filtered(
        bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (message == nullptr) {
      return true;
    }
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS) {
      gst_message_unref(message);
      DispatchGStreamerWarning("GStreamer video stream ended; reconnecting.");
      return false;
    }
    GError *error = nullptr;
    gchar *debug = nullptr;
    gst_message_parse_error(message, &error, &debug);
    const std::string message_text =
        error != nullptr ? error->message : "unknown video pipeline error";
    if (debug != nullptr) {
      g_free(debug);
    }
    if (error != nullptr) {
      g_error_free(error);
    }
    gst_message_unref(message);
    DispatchGStreamerWarning("GStreamer video stream failed: " + message_text +
                             "; reconnecting.");
    return false;
  }

  GstSample *pull_latest_sample() {
    if (test_sink_ == nullptr) {
      return nullptr;
    }
    GstSample *sample = nullptr;
    while (GstSample *candidate = gst_app_sink_try_pull_sample(test_sink_, 0)) {
      if (sample != nullptr) {
        gst_sample_unref(sample);
      }
      sample = candidate;
    }
    return sample;
  }

  void configure_video_format(GstSample *sample, const bool initial) {
    GstCaps *caps = gst_sample_get_caps(sample);
    GstVideoInfo info{};
    if (caps == nullptr || !gst_video_info_from_caps(&info, caps) ||
        GST_VIDEO_INFO_FORMAT(&info) != GST_VIDEO_FORMAT_RGBA) {
      throw std::runtime_error(
          "GStreamer video source did not negotiate RGBA video caps");
    }

    const uint32_t source_width = GST_VIDEO_INFO_WIDTH(&info);
    const uint32_t source_height = GST_VIDEO_INFO_HEIGHT(&info);
    if (source_width == 0 || source_height == 0 ||
        (video_type_ == sawOpenXR::VideoType::SideBySide &&
         source_width % 2 != 0)) {
      throw std::runtime_error(
          "invalid negotiated dimensions for the configured video.type");
    }
    const uint32_t eye_width =
        video_type_ == sawOpenXR::VideoType::Mono ? source_width
                                                  : source_width / 2;

    if (!initial &&
        (eye_width != eye_video_width_ || source_height != eye_video_height_)) {
      throw std::runtime_error(
          "reconnected video source changed resolution; restart sawOpenXR "
          "to recreate its Vulkan video textures");
    }

    test_video_info_ = info;
    source_video_width_ = source_width;
    eye_video_width_ = eye_width;
    eye_video_height_ = source_height;
    std::ostringstream message;
    message << "GStreamer video: " << source_video_width_ << "x"
            << eye_video_height_ << " "
            << (video_type_ == sawOpenXR::VideoType::Mono ? "mono"
                                                           : "side-by-side")
            << "; OpenXR eye image " << eye_video_width_ << "x"
            << eye_video_height_ << ".";
    DispatchGStreamerStatus(message.str());
  }

  void schedule_video_reconnect() {
    stop_test_source();
    video_format_checked_ = false;
    next_video_retry_ =
        std::chrono::steady_clock::now() + kVideoReconnectDelay;
  }

  void wait_for_initial_video_frame() {
    DispatchGStreamerStatus("Waiting for the configured video source...");
    while (!stop_requested_.load()) {
      if (test_pipeline_ == nullptr &&
          std::chrono::steady_clock::now() >= next_video_retry_) {
        if (!start_test_source()) {
          next_video_retry_ =
              std::chrono::steady_clock::now() + kVideoReconnectDelay;
        }
      }
      if (test_pipeline_ != nullptr && !check_stream_bus()) {
        schedule_video_reconnect();
      }
      if (GstSample *sample = pull_latest_sample()) {
        configure_video_format(sample, true);
        video_format_checked_ = true;
        gst_sample_unref(sample);
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void create_instance() {
    const char *extensions[] = {XR_KHR_VULKAN_ENABLE2_EXTENSION_NAME};
    XrInstanceCreateInfo info{XR_TYPE_INSTANCE_CREATE_INFO};
    std::strncpy(info.applicationInfo.applicationName, "Quest OpenXR POC",
                 XR_MAX_APPLICATION_NAME_SIZE - 1);
    info.applicationInfo.applicationVersion = 1;
    std::strncpy(info.applicationInfo.engineName, "none",
                 XR_MAX_ENGINE_NAME_SIZE - 1);
    info.applicationInfo.apiVersion = XR_CURRENT_API_VERSION;
    info.enabledExtensionCount = 1;
    info.enabledExtensionNames = extensions;
    XR_CHECK(xrCreateInstance(&info, &instance_));

    XrInstanceProperties properties{XR_TYPE_INSTANCE_PROPERTIES};
    XR_CHECK(xrGetInstanceProperties(instance_, &properties));
    std::cout << "OpenXR runtime: " << properties.runtimeName << " "
              << XR_VERSION_MAJOR(properties.runtimeVersion) << "."
              << XR_VERSION_MINOR(properties.runtimeVersion) << "."
              << XR_VERSION_PATCH(properties.runtimeVersion) << "\n";

    XR_CHECK(xrGetInstanceProcAddr(
        instance_, "xrGetVulkanGraphicsRequirements2KHR",
        reinterpret_cast<PFN_xrVoidFunction *>(&get_graphics_requirements_)));
    XR_CHECK(xrGetInstanceProcAddr(
        instance_, "xrCreateVulkanInstanceKHR",
        reinterpret_cast<PFN_xrVoidFunction *>(&create_vulkan_instance_)));
    XR_CHECK(xrGetInstanceProcAddr(
        instance_, "xrGetVulkanGraphicsDevice2KHR",
        reinterpret_cast<PFN_xrVoidFunction *>(&get_graphics_device_)));
    XR_CHECK(xrGetInstanceProcAddr(
        instance_, "xrCreateVulkanDeviceKHR",
        reinterpret_cast<PFN_xrVoidFunction *>(&create_vulkan_device_)));

    XrSystemGetInfo system_info{XR_TYPE_SYSTEM_GET_INFO};
    system_info.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    XR_CHECK(xrGetSystem(instance_, &system_info, &system_id_));
    XrSystemProperties system_properties{XR_TYPE_SYSTEM_PROPERTIES};
    XR_CHECK(xrGetSystemProperties(instance_, system_id_, &system_properties));
    std::cout << "XR system: " << system_properties.systemName << "\n";
  }

  void create_session() {
    XrGraphicsRequirementsVulkanKHR requirements{
        XR_TYPE_GRAPHICS_REQUIREMENTS_VULKAN_KHR};
    std::cerr << "Querying Vulkan requirements...\n";
    XR_CHECK(get_graphics_requirements_(instance_, system_id_, &requirements));

    VkApplicationInfo app_info{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app_info.pApplicationName = "Quest OpenXR POC";
    // OpenXR stores the Vulkan version in an XrVersion-sized field. Let
    // Vulkan/OpenXR validate the actual supported range instead of
    // comparing the two APIs' differently encoded version values here.
    app_info.apiVersion = VK_API_VERSION_1_3;
    VkInstanceCreateInfo instance_info{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    instance_info.pApplicationInfo = &app_info;
    std::cerr << "Creating Vulkan instance...\n";
    XrVulkanInstanceCreateInfoKHR xr_instance_info{
        XR_TYPE_VULKAN_INSTANCE_CREATE_INFO_KHR};
    xr_instance_info.systemId = system_id_;
    xr_instance_info.pfnGetInstanceProcAddr = vkGetInstanceProcAddr;
    xr_instance_info.vulkanCreateInfo = &instance_info;
    VkResult vulkan_result = VK_SUCCESS;
    XR_CHECK(create_vulkan_instance_(instance_, &xr_instance_info,
                                     &vk_.instance, &vulkan_result));
    VK_CHECK(vulkan_result);

    std::cerr << "Selecting Vulkan device...\n";
    XrVulkanGraphicsDeviceGetInfoKHR device_get_info{
        XR_TYPE_VULKAN_GRAPHICS_DEVICE_GET_INFO_KHR};
    device_get_info.systemId = system_id_;
    device_get_info.vulkanInstance = vk_.instance;
    XR_CHECK(get_graphics_device_(instance_, &device_get_info,
                                  &vk_.physical_device));

    uint32_t queue_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(vk_.physical_device, &queue_count,
                                             nullptr);
    std::vector<VkQueueFamilyProperties> queues(queue_count);
    vkGetPhysicalDeviceQueueFamilyProperties(vk_.physical_device, &queue_count,
                                             queues.data());
    for (uint32_t i = 0; i < queue_count; ++i) {
      if ((queues[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0) {
        vk_.queue_family = i;
        break;
      }
    }

    const float priority = 1.0f;
    VkDeviceQueueCreateInfo queue_info{
        VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
    queue_info.queueFamilyIndex = vk_.queue_family;
    queue_info.queueCount = 1;
    queue_info.pQueuePriorities = &priority;
    VkDeviceCreateInfo create_info{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    create_info.queueCreateInfoCount = 1;
    create_info.pQueueCreateInfos = &queue_info;
    std::cerr << "Creating Vulkan device...\n";
    XrVulkanDeviceCreateInfoKHR xr_device_info{
        XR_TYPE_VULKAN_DEVICE_CREATE_INFO_KHR};
    xr_device_info.systemId = system_id_;
    xr_device_info.pfnGetInstanceProcAddr = vkGetInstanceProcAddr;
    xr_device_info.vulkanPhysicalDevice = vk_.physical_device;
    xr_device_info.vulkanCreateInfo = &create_info;
    XR_CHECK(create_vulkan_device_(instance_, &xr_device_info, &vk_.device,
                                   &vulkan_result));
    VK_CHECK(vulkan_result);
    vkGetDeviceQueue(vk_.device, vk_.queue_family, 0, &vk_.queue);

    VkCommandPoolCreateInfo pool_info{
        VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    pool_info.queueFamilyIndex = vk_.queue_family;
    VK_CHECK(vkCreateCommandPool(vk_.device, &pool_info, nullptr,
                                 &vk_.command_pool));
    VkCommandBufferAllocateInfo command_info{
        VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    command_info.commandPool = vk_.command_pool;
    command_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    command_info.commandBufferCount = 1;
    VK_CHECK(vkAllocateCommandBuffers(vk_.device, &command_info,
                                      &vk_.command_buffer));
    VkFenceCreateInfo fence_info{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    // clear_image waits before reusing this command buffer. The very first
    // frame has no prior submission, so start in the signaled state.
    fence_info.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    VK_CHECK(vkCreateFence(vk_.device, &fence_info, nullptr, &vk_.fence));

    XrGraphicsBindingVulkan2KHR binding{XR_TYPE_GRAPHICS_BINDING_VULKAN2_KHR};
    binding.instance = vk_.instance;
    binding.physicalDevice = vk_.physical_device;
    binding.device = vk_.device;
    binding.queueFamilyIndex = vk_.queue_family;
    binding.queueIndex = 0;
    XrSessionCreateInfo session_info{XR_TYPE_SESSION_CREATE_INFO};
    session_info.next = &binding;
    session_info.systemId = system_id_;
    std::cerr << "Creating OpenXR session...\n";
    XR_CHECK(xrCreateSession(instance_, &session_info, &session_));

    uint32_t view_count = 0;
    XR_CHECK(xrEnumerateViewConfigurationViews(
        instance_, system_id_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0,
        &view_count, nullptr));
    config_views_.assign(view_count, {XR_TYPE_VIEW_CONFIGURATION_VIEW});
    XR_CHECK(xrEnumerateViewConfigurationViews(
        instance_, system_id_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
        view_count, &view_count, config_views_.data()));
    width_ = config_views_[0].recommendedImageRectWidth;
    height_ = config_views_[0].recommendedImageRectHeight;
    views_.assign(view_count, {XR_TYPE_VIEW});

    uint32_t blend_mode_count = 0;
    XR_CHECK(xrEnumerateEnvironmentBlendModes(
        instance_, system_id_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 0,
        &blend_mode_count, nullptr));
    std::vector<XrEnvironmentBlendMode> blend_modes(blend_mode_count);
    XR_CHECK(xrEnumerateEnvironmentBlendModes(
        instance_, system_id_, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO,
        blend_mode_count, &blend_mode_count, blend_modes.data()));
    if (std::find(blend_modes.begin(), blend_modes.end(),
                  XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND) != blend_modes.end()) {
      environment_blend_mode_ = XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND;
      std::cout << "OpenXR alpha blend is available: requesting transparent "
                   "background / room passthrough.\n";
    } else {
      std::cerr << "OpenXR runtime supports only an opaque background; room "
                   "passthrough is unavailable.\n";
    }

    XrReferenceSpaceCreateInfo space_info{XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
    space_info.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    space_info.poseInReferenceSpace.orientation.w = 1.0f;
    XR_CHECK(xrCreateReferenceSpace(session_, &space_info, &local_space_));

    uint32_t format_count = 0;
    XR_CHECK(xrEnumerateSwapchainFormats(session_, 0, &format_count, nullptr));
    std::vector<int64_t> formats(format_count);
    XR_CHECK(xrEnumerateSwapchainFormats(session_, format_count, &format_count,
                                         formats.data()));
    const auto preferred = static_cast<int64_t>(VK_FORMAT_R8G8B8A8_SRGB);
    color_format_ =
        std::find(formats.begin(), formats.end(), preferred) != formats.end()
            ? preferred
            : formats.front();

    create_render_pass();
    create_video_resources();

    XrSwapchainCreateInfo swapchain_info{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    swapchain_info.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
    swapchain_info.format = color_format_;
    swapchain_info.sampleCount = 1;
    swapchain_info.width = width_;
    swapchain_info.height = height_;
    swapchain_info.faceCount = 1;
    swapchain_info.arraySize = 1;
    swapchain_info.mipCount = 1;
    eye_swapchains_.resize(view_count);
    for (auto &eye : eye_swapchains_) {
      XR_CHECK(xrCreateSwapchain(session_, &swapchain_info, &eye.handle));
      uint32_t image_count = 0;
      XR_CHECK(
          xrEnumerateSwapchainImages(eye.handle, 0, &image_count, nullptr));
      std::vector<XrSwapchainImageVulkan2KHR> xr_images(
          image_count, {XR_TYPE_SWAPCHAIN_IMAGE_VULKAN2_KHR});
      XR_CHECK(xrEnumerateSwapchainImages(
          eye.handle, image_count, &image_count,
          reinterpret_cast<XrSwapchainImageBaseHeader *>(xr_images.data())));
      for (const auto &image : xr_images) {
        eye.images.push_back(image.image);

        VkImageViewCreateInfo image_view_info{
            VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        image_view_info.image = image.image;
        image_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
        image_view_info.format = static_cast<VkFormat>(color_format_);
        image_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        image_view_info.subresourceRange.levelCount = 1;
        image_view_info.subresourceRange.layerCount = 1;
        VkImageView image_view = VK_NULL_HANDLE;
        VK_CHECK(vkCreateImageView(vk_.device, &image_view_info, nullptr,
                                   &image_view));
        eye.image_views.push_back(image_view);

        VkFramebufferCreateInfo framebuffer_info{
            VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        framebuffer_info.renderPass = render_pass_;
        framebuffer_info.attachmentCount = 1;
        framebuffer_info.pAttachments = &eye.image_views.back();
        framebuffer_info.width = width_;
        framebuffer_info.height = height_;
        framebuffer_info.layers = 1;
        VkFramebuffer framebuffer = VK_NULL_HANDLE;
        VK_CHECK(vkCreateFramebuffer(vk_.device, &framebuffer_info, nullptr,
                                     &framebuffer));
        eye.framebuffers.push_back(framebuffer);
      }
    }

    create_actions();
  }

  void create_render_pass() {
    VkAttachmentDescription color_attachment{};
    color_attachment.format = static_cast<VkFormat>(color_format_);
    color_attachment.samples = VK_SAMPLE_COUNT_1_BIT;
    color_attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color_attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color_attachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color_attachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color_attachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color_attachment.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference color_reference{};
    color_reference.attachment = 0;
    color_reference.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &color_reference;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo info{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    info.attachmentCount = 1;
    info.pAttachments = &color_attachment;
    info.subpassCount = 1;
    info.pSubpasses = &subpass;
    info.dependencyCount = 1;
    info.pDependencies = &dependency;
    VK_CHECK(vkCreateRenderPass(vk_.device, &info, nullptr, &render_pass_));
  }

  uint32_t find_memory_type(uint32_t type_bits,
                            VkMemoryPropertyFlags required_properties) const {
    VkPhysicalDeviceMemoryProperties properties{};
    vkGetPhysicalDeviceMemoryProperties(vk_.physical_device, &properties);
    for (uint32_t i = 0; i < properties.memoryTypeCount; ++i) {
      if ((type_bits & (1u << i)) != 0 &&
          (properties.memoryTypes[i].propertyFlags & required_properties) ==
              required_properties) {
        return i;
      }
    }
    throw std::runtime_error(
        "No Vulkan memory type satisfies the requested properties");
  }

  static std::vector<uint32_t> read_spirv(const std::string &filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file) {
      throw std::runtime_error("Could not open shader " + filename);
    }
    const auto byte_count = file.tellg();
    if (byte_count <= 0 || (byte_count % 4) != 0) {
      throw std::runtime_error("Invalid SPIR-V shader " + filename);
    }
    std::vector<uint32_t> code(static_cast<size_t>(byte_count) /
                               sizeof(uint32_t));
    file.seekg(0);
    file.read(reinterpret_cast<char *>(code.data()), byte_count);
    if (!file) {
      throw std::runtime_error("Could not read shader " + filename);
    }
    return code;
  }

  VkShaderModule create_shader_module(const std::string &filename) const {
    const auto code = read_spirv(filename);
    VkShaderModuleCreateInfo info{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    info.codeSize = code.size() * sizeof(uint32_t);
    info.pCode = code.data();
    VkShaderModule shader = VK_NULL_HANDLE;
    VK_CHECK(vkCreateShaderModule(vk_.device, &info, nullptr, &shader));
    return shader;
  }

  void create_video_resources() {
    const VkDeviceSize frame_size =
        static_cast<VkDeviceSize>(eye_video_width_) * eye_video_height_ * 4;
    VkBufferCreateInfo buffer_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    buffer_info.size = frame_size;
    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
    buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VkImageCreateInfo image_info{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    image_info.imageType = VK_IMAGE_TYPE_2D;
    image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
    image_info.extent = {eye_video_width_, eye_video_height_, 1};
    image_info.mipLevels = 1;
    image_info.arrayLayers = 1;
    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
    image_info.usage =
        VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    image_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    for (auto &texture : video_textures_) {
      VK_CHECK(vkCreateBuffer(vk_.device, &buffer_info, nullptr,
                              &texture.staging_buffer));
      VkMemoryRequirements staging_requirements{};
      vkGetBufferMemoryRequirements(vk_.device, texture.staging_buffer,
                                    &staging_requirements);
      VkMemoryAllocateInfo staging_allocation{
          VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
      staging_allocation.allocationSize = staging_requirements.size;
      staging_allocation.memoryTypeIndex =
          find_memory_type(staging_requirements.memoryTypeBits,
                           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                               VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
      VK_CHECK(vkAllocateMemory(vk_.device, &staging_allocation, nullptr,
                                &texture.staging_memory));
      VK_CHECK(vkBindBufferMemory(vk_.device, texture.staging_buffer,
                                  texture.staging_memory, 0));
      VK_CHECK(vkMapMemory(vk_.device, texture.staging_memory, 0, frame_size, 0,
                           &texture.staging_mapped));

      VK_CHECK(vkCreateImage(vk_.device, &image_info, nullptr, &texture.image));
      VkMemoryRequirements image_requirements{};
      vkGetImageMemoryRequirements(vk_.device, texture.image,
                                   &image_requirements);
      VkMemoryAllocateInfo image_allocation{
          VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
      image_allocation.allocationSize = image_requirements.size;
      image_allocation.memoryTypeIndex =
          find_memory_type(image_requirements.memoryTypeBits,
                           VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
      VK_CHECK(vkAllocateMemory(vk_.device, &image_allocation, nullptr,
                                &texture.image_memory));
      VK_CHECK(vkBindImageMemory(vk_.device, texture.image,
                                 texture.image_memory, 0));

      VkImageViewCreateInfo image_view_info{
          VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
      image_view_info.image = texture.image;
      image_view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
      image_view_info.format = VK_FORMAT_R8G8B8A8_UNORM;
      image_view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      image_view_info.subresourceRange.levelCount = 1;
      image_view_info.subresourceRange.layerCount = 1;
      VK_CHECK(vkCreateImageView(vk_.device, &image_view_info, nullptr,
                                 &texture.image_view));
    }

    VkSamplerCreateInfo sampler_info{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sampler_info.magFilter = VK_FILTER_LINEAR;
    sampler_info.minFilter = VK_FILTER_LINEAR;
    sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sampler_info.maxLod = 0.0f;
    VK_CHECK(
        vkCreateSampler(vk_.device, &sampler_info, nullptr, &test_sampler_));

    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo layout_info{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    layout_info.bindingCount = 1;
    layout_info.pBindings = &binding;
    VK_CHECK(vkCreateDescriptorSetLayout(vk_.device, &layout_info, nullptr,
                                         &test_descriptor_set_layout_));
    VkDescriptorPoolSize pool_size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                   eye_count_};
    VkDescriptorPoolCreateInfo pool_info{
        VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    pool_info.maxSets = eye_count_;
    pool_info.poolSizeCount = 1;
    pool_info.pPoolSizes = &pool_size;
    VK_CHECK(vkCreateDescriptorPool(vk_.device, &pool_info, nullptr,
                                    &test_descriptor_pool_));
    VkDescriptorSetAllocateInfo descriptor_allocate{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    descriptor_allocate.descriptorPool = test_descriptor_pool_;
    const std::array<VkDescriptorSetLayout, eye_count_> set_layouts{
        test_descriptor_set_layout_, test_descriptor_set_layout_};
    std::array<VkDescriptorSet, eye_count_> descriptor_sets{};
    descriptor_allocate.descriptorSetCount = eye_count_;
    descriptor_allocate.pSetLayouts = set_layouts.data();
    VK_CHECK(vkAllocateDescriptorSets(vk_.device, &descriptor_allocate,
                                      descriptor_sets.data()));
    for (uint32_t eye = 0; eye < eye_count_; ++eye) {
      auto &texture = video_textures_[eye];
      texture.descriptor_set = descriptor_sets[eye];
      VkDescriptorImageInfo descriptor_image{};
      descriptor_image.sampler = test_sampler_;
      descriptor_image.imageView = texture.image_view;
      descriptor_image.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      VkWriteDescriptorSet descriptor_write{
          VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
      descriptor_write.dstSet = texture.descriptor_set;
      descriptor_write.dstBinding = 0;
      descriptor_write.descriptorCount = 1;
      descriptor_write.descriptorType =
          VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
      descriptor_write.pImageInfo = &descriptor_image;
      vkUpdateDescriptorSets(vk_.device, 1, &descriptor_write, 0, nullptr);
    }

    VkPipelineLayoutCreateInfo pipeline_layout_info{
        VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pipeline_layout_info.setLayoutCount = 1;
    pipeline_layout_info.pSetLayouts = &test_descriptor_set_layout_;
    VkPushConstantRange transform_range{};
    transform_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    transform_range.offset = 0;
    transform_range.size = sizeof(Mat4);
    pipeline_layout_info.pushConstantRangeCount = 1;
    pipeline_layout_info.pPushConstantRanges = &transform_range;
    VK_CHECK(vkCreatePipelineLayout(vk_.device, &pipeline_layout_info, nullptr,
                                    &test_pipeline_layout_));
    VkShaderModule vertex_shader = create_shader_module(
        std::string(SAW_OPENXR_SHADER_DIR) + "/texture.vert.spv");
    VkShaderModule fragment_shader = create_shader_module(
        std::string(SAW_OPENXR_SHADER_DIR) + "/texture.frag.spv");
    const std::array<VkPipelineShaderStageCreateInfo, 2> stages{{
        {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0,
         VK_SHADER_STAGE_VERTEX_BIT, vertex_shader, "main"},
        {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO, nullptr, 0,
         VK_SHADER_STAGE_FRAGMENT_BIT, fragment_shader, "main"},
    }};
    VkPipelineVertexInputStateCreateInfo vertex_input{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    VkPipelineInputAssemblyStateCreateInfo input_assembly{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkViewport viewport{
        0.0f, 0.0f, static_cast<float>(width_), static_cast<float>(height_),
        0.0f, 1.0f};
    VkRect2D scissor{{0, 0}, {width_, height_}};
    VkPipelineViewportStateCreateInfo viewport_state{
        VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    viewport_state.viewportCount = 1;
    viewport_state.pViewports = &viewport;
    viewport_state.scissorCount = 1;
    viewport_state.pScissors = &scissor;
    VkPipelineRasterizationStateCreateInfo rasterization{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rasterization.polygonMode = VK_POLYGON_MODE_FILL;
    rasterization.cullMode = VK_CULL_MODE_NONE;
    rasterization.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rasterization.lineWidth = 1.0f;
    VkPipelineMultisampleStateCreateInfo multisample{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineColorBlendAttachmentState blend_attachment{};
    blend_attachment.colorWriteMask =
        VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo color_blend{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    color_blend.attachmentCount = 1;
    color_blend.pAttachments = &blend_attachment;
    VkGraphicsPipelineCreateInfo graphics_info{
        VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    graphics_info.stageCount = static_cast<uint32_t>(stages.size());
    graphics_info.pStages = stages.data();
    graphics_info.pVertexInputState = &vertex_input;
    graphics_info.pInputAssemblyState = &input_assembly;
    graphics_info.pViewportState = &viewport_state;
    graphics_info.pRasterizationState = &rasterization;
    graphics_info.pMultisampleState = &multisample;
    graphics_info.pColorBlendState = &color_blend;
    graphics_info.layout = test_pipeline_layout_;
    graphics_info.renderPass = render_pass_;
    VK_CHECK(vkCreateGraphicsPipelines(vk_.device, VK_NULL_HANDLE, 1,
                                       &graphics_info, nullptr,
                                       &test_graphics_pipeline_));
    vkDestroyShaderModule(vk_.device, fragment_shader, nullptr);
    vkDestroyShaderModule(vk_.device, vertex_shader, nullptr);
  }

  void destroy_video_resources() {
    if (vk_.device == VK_NULL_HANDLE) {
      return;
    }
    if (test_graphics_pipeline_ != VK_NULL_HANDLE) {
      vkDestroyPipeline(vk_.device, test_graphics_pipeline_, nullptr);
    }
    if (test_pipeline_layout_ != VK_NULL_HANDLE) {
      vkDestroyPipelineLayout(vk_.device, test_pipeline_layout_, nullptr);
    }
    if (test_descriptor_pool_ != VK_NULL_HANDLE) {
      vkDestroyDescriptorPool(vk_.device, test_descriptor_pool_, nullptr);
    }
    if (test_descriptor_set_layout_ != VK_NULL_HANDLE) {
      vkDestroyDescriptorSetLayout(vk_.device, test_descriptor_set_layout_,
                                   nullptr);
    }
    if (test_sampler_ != VK_NULL_HANDLE) {
      vkDestroySampler(vk_.device, test_sampler_, nullptr);
    }
    for (auto &texture : video_textures_) {
      if (texture.image_view != VK_NULL_HANDLE) {
        vkDestroyImageView(vk_.device, texture.image_view, nullptr);
      }
      if (texture.image != VK_NULL_HANDLE) {
        vkDestroyImage(vk_.device, texture.image, nullptr);
      }
      if (texture.image_memory != VK_NULL_HANDLE) {
        vkFreeMemory(vk_.device, texture.image_memory, nullptr);
      }
      if (texture.staging_mapped != nullptr) {
        vkUnmapMemory(vk_.device, texture.staging_memory);
      }
      if (texture.staging_buffer != VK_NULL_HANDLE) {
        vkDestroyBuffer(vk_.device, texture.staging_buffer, nullptr);
      }
      if (texture.staging_memory != VK_NULL_HANDLE) {
        vkFreeMemory(vk_.device, texture.staging_memory, nullptr);
      }
    }
  }

  XrPath path(const char *value) const {
    XrPath result = XR_NULL_PATH;
    XR_CHECK(xrStringToPath(instance_, value, &result));
    return result;
  }

  void create_actions() {
    hand_paths_[0] = path("/user/hand/left");
    hand_paths_[1] = path("/user/hand/right");
    XrActionSetCreateInfo set_info{XR_TYPE_ACTION_SET_CREATE_INFO};
    std::strncpy(set_info.actionSetName, "main",
                 XR_MAX_ACTION_SET_NAME_SIZE - 1);
    std::strncpy(set_info.localizedActionSetName, "Main",
                 XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE - 1);
    XR_CHECK(xrCreateActionSet(instance_, &set_info, &action_set_));

    XrActionCreateInfo action_info{XR_TYPE_ACTION_CREATE_INFO};
    action_info.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
    std::strncpy(action_info.actionName, "quit", XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(action_info.localizedActionName, "Quit",
                 XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    XR_CHECK(xrCreateAction(action_set_, &action_info, &quit_action_));

    action_info = {XR_TYPE_ACTION_CREATE_INFO};
    action_info.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
    std::strncpy(action_info.actionName, "move_window",
                 XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(action_info.localizedActionName, "Move video window",
                 XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    action_info.countSubactionPaths = static_cast<uint32_t>(hand_paths_.size());
    action_info.subactionPaths = hand_paths_.data();
    XR_CHECK(xrCreateAction(action_set_, &action_info, &grab_action_));

    action_info = {XR_TYPE_ACTION_CREATE_INFO};
    action_info.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
    std::strncpy(action_info.actionName, "reset_window",
                 XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(action_info.localizedActionName, "Reset video window",
                 XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    XR_CHECK(xrCreateAction(action_set_, &action_info, &reset_window_action_));

    action_info = {XR_TYPE_ACTION_CREATE_INFO};
    action_info.actionType = XR_ACTION_TYPE_VECTOR2F_INPUT;
    std::strncpy(action_info.actionName, "thumbstick",
                 XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(action_info.localizedActionName, "Thumbstick",
                 XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    action_info.countSubactionPaths = static_cast<uint32_t>(hand_paths_.size());
    action_info.subactionPaths = hand_paths_.data();
    XR_CHECK(xrCreateAction(action_set_, &action_info, &thumbstick_action_));

    action_info = {XR_TYPE_ACTION_CREATE_INFO};
    action_info.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
    std::strncpy(action_info.actionName, "thumbstick_click",
                 XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(action_info.localizedActionName, "Thumbstick click",
                 XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    action_info.countSubactionPaths = static_cast<uint32_t>(hand_paths_.size());
    action_info.subactionPaths = hand_paths_.data();
    XR_CHECK(
        xrCreateAction(action_set_, &action_info, &thumbstick_click_action_));

    action_info = {XR_TYPE_ACTION_CREATE_INFO};
    action_info.actionType = XR_ACTION_TYPE_FLOAT_INPUT;
    std::strncpy(action_info.actionName, "front_trigger",
                 XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(action_info.localizedActionName, "Front trigger",
                 XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    action_info.countSubactionPaths = static_cast<uint32_t>(hand_paths_.size());
    action_info.subactionPaths = hand_paths_.data();
    XR_CHECK(xrCreateAction(action_set_, &action_info, &front_trigger_action_));

    action_info = {XR_TYPE_ACTION_CREATE_INFO};
    action_info.actionType = XR_ACTION_TYPE_POSE_INPUT;
    std::strncpy(action_info.actionName, "grip_pose",
                 XR_MAX_ACTION_NAME_SIZE - 1);
    std::strncpy(action_info.localizedActionName, "Grip pose",
                 XR_MAX_LOCALIZED_ACTION_NAME_SIZE - 1);
    action_info.countSubactionPaths = static_cast<uint32_t>(hand_paths_.size());
    action_info.subactionPaths = hand_paths_.data();
    XR_CHECK(xrCreateAction(action_set_, &action_info, &grip_pose_action_));

    const std::array<XrActionSuggestedBinding, 11> bindings{{
        {quit_action_, path("/user/hand/left/input/menu/click")},
        {grab_action_, path("/user/hand/right/input/a/click")},
        {reset_window_action_, path("/user/hand/left/input/x/click")},
        {thumbstick_action_, path("/user/hand/left/input/thumbstick")},
        {thumbstick_action_, path("/user/hand/right/input/thumbstick")},
        {thumbstick_click_action_,
         path("/user/hand/left/input/thumbstick/click")},
        {thumbstick_click_action_,
         path("/user/hand/right/input/thumbstick/click")},
        {front_trigger_action_, path("/user/hand/left/input/trigger/value")},
        {front_trigger_action_, path("/user/hand/right/input/trigger/value")},
        {grip_pose_action_, path("/user/hand/left/input/grip/pose")},
        {grip_pose_action_, path("/user/hand/right/input/grip/pose")},
    }};
    XrInteractionProfileSuggestedBinding suggested{
        XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING};
    suggested.interactionProfile =
        path("/interaction_profiles/oculus/touch_controller");
    suggested.suggestedBindings = bindings.data();
    suggested.countSuggestedBindings = static_cast<uint32_t>(bindings.size());
    XR_CHECK(xrSuggestInteractionProfileBindings(instance_, &suggested));

    for (uint32_t hand = 0; hand < hand_paths_.size(); ++hand) {
      XrActionSpaceCreateInfo grip_space_info{XR_TYPE_ACTION_SPACE_CREATE_INFO};
      grip_space_info.action = grip_pose_action_;
      grip_space_info.subactionPath = hand_paths_[hand];
      grip_space_info.poseInActionSpace.orientation.w = 1.0f;
      XR_CHECK(
          xrCreateActionSpace(session_, &grip_space_info, &grip_spaces_[hand]));
    }

    XrSessionActionSetsAttachInfo attach{
        XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO};
    attach.countActionSets = 1;
    attach.actionSets = &action_set_;
    XR_CHECK(xrAttachSessionActionSets(session_, &attach));
    std::cerr << "Hold right A and move the controller to position the video "
                 "window. Press left X to reset it in front of you; press the "
                 "left Menu button to exit.\n";
  }

  void poll_events() {
    XrEventDataBuffer event{XR_TYPE_EVENT_DATA_BUFFER};
    while (xrPollEvent(instance_, &event) == XR_SUCCESS) {
      if (event.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
        const auto &changed =
            reinterpret_cast<const XrEventDataSessionStateChanged &>(event);
        if (changed.state == XR_SESSION_STATE_READY) {
          std::cerr << "OpenXR session is ready.\n";
          XrSessionBeginInfo begin{XR_TYPE_SESSION_BEGIN_INFO};
          begin.primaryViewConfigurationType =
              XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
          XR_CHECK(xrBeginSession(session_, &begin));
          session_running_ = true;
          std::cerr << "OpenXR session started.\n";
        } else if (changed.state == XR_SESSION_STATE_STOPPING) {
          XR_CHECK(xrEndSession(session_));
          session_running_ = false;
        } else if (changed.state == XR_SESSION_STATE_EXITING ||
                   changed.state == XR_SESSION_STATE_LOSS_PENDING) {
          exit_requested_ = true;
        }
      } else if (event.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
        exit_requested_ = true;
      }
      event = {XR_TYPE_EVENT_DATA_BUFFER};
    }
  }

  bool pull_test_frame() {
    if (test_pipeline_ == nullptr) {
      if (std::chrono::steady_clock::now() < next_video_retry_ ||
          !start_test_source()) {
        next_video_retry_ =
            std::chrono::steady_clock::now() + kVideoReconnectDelay;
        return false;
      }
    }
    if (!check_stream_bus()) {
      schedule_video_reconnect();
      return false;
    }

    // Keep only the most recent sample even if an older binary or GStreamer
    // implementation allowed more than one sample to reach appsink.
    GstSample *sample = pull_latest_sample();

    if (sample == nullptr) {
      return false;
    }

    if (!video_format_checked_) {
      configure_video_format(sample, false);
      video_format_checked_ = true;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);

    latest_video_age_valid_ = false;
    sender_video_age_valid_ = false;

    if (buffer != nullptr) {
      const GstReferenceTimestampMeta *reference_meta =
          gst_buffer_get_reference_timestamp_meta(buffer, nullptr);

      if (reference_meta != nullptr) {
        const GstClockTime unix_time = static_cast<GstClockTime>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        GstClockTime reference_now = GST_CLOCK_TIME_NONE;

        if (gst_caps_get_size(reference_meta->reference) > 0) {
          const GstStructure *reference_structure =
              gst_caps_get_structure(reference_meta->reference, 0);
          const gchar *reference_name =
              gst_structure_get_name(reference_structure);

          if (g_strcmp0(reference_name, "timestamp/x-unix") == 0) {
            reference_now = unix_time;
          } else if (g_strcmp0(reference_name, "timestamp/x-ntp") == 0) {
            constexpr GstClockTime ntp_to_unix_seconds = 2208988800ULL;
            reference_now = unix_time + ntp_to_unix_seconds * GST_SECOND;
          }
        }

        if (GST_CLOCK_TIME_IS_VALID(reference_now) &&
            reference_now >= reference_meta->timestamp) {
          sender_video_age_ms_ =
              static_cast<double>(reference_now - reference_meta->timestamp) /
              static_cast<double>(GST_MSECOND);
          sender_video_age_valid_ = true;
        }

        if (!sender_timestamp_logged_) {
          gchar *reference = gst_caps_to_string(reference_meta->reference);
          std::cout << "Video sender timestamp metadata: "
                    << (reference != nullptr ? reference : "unknown")
                    << std::endl;
          g_free(reference);
          sender_timestamp_logged_ = true;
        }
      }
    }

    if (buffer != nullptr && GST_BUFFER_PTS_IS_VALID(buffer)) {
      const GstSegment *segment = gst_sample_get_segment(sample);
      const GstClockTime buffer_running_time = gst_segment_to_running_time(
          segment, GST_FORMAT_TIME, GST_BUFFER_PTS(buffer));
      GstClock *clock = gst_element_get_clock(test_pipeline_);

      if (clock != nullptr && GST_CLOCK_TIME_IS_VALID(buffer_running_time)) {
        const GstClockTime clock_time = gst_clock_get_time(clock);
        const GstClockTime base_time =
            gst_element_get_base_time(test_pipeline_);

        if (clock_time >= base_time) {
          const GstClockTime pipeline_running_time = clock_time - base_time;
          const GstClockTimeDiff age =
              GST_CLOCK_DIFF(buffer_running_time, pipeline_running_time);
          latest_video_age_ms_ =
              static_cast<double>(age) / static_cast<double>(GST_MSECOND);
          latest_video_age_valid_ = true;
        }

        gst_object_unref(clock);
      }
    }

    GstVideoFrame frame{};
    const bool mapped =
        buffer != nullptr &&
        gst_video_frame_map(&frame, &test_video_info_, buffer, GST_MAP_READ);
    if (!mapped) {
      gst_sample_unref(sample);
      DispatchGStreamerWarning(
          "GStreamer produced an unexpected test frame; waiting for the next one.");
      return false;
    }
    const auto *source =
        static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    const int source_stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    const size_t row_size = static_cast<size_t>(eye_video_width_) * 4;
    for (uint32_t y = 0; y < eye_video_height_; ++y) {
      const auto *source_row = source + y * source_stride;
      std::memcpy(static_cast<uint8_t *>(video_textures_[0].staging_mapped) +
                      y * row_size,
                  source_row, row_size);
      std::memcpy(static_cast<uint8_t *>(video_textures_[1].staging_mapped) +
                      y * row_size,
                  source_row +
                      (video_type_ == sawOpenXR::VideoType::Mono ? 0
                                                                 : row_size),
                  row_size);
    }
    gst_video_frame_unmap(&frame);
    gst_sample_unref(sample);
    has_test_frame_ = true;
    ++test_frame_count_;
    return true;
  }

  void transition_test_image(VkCommandBuffer command_buffer,
                             const VideoTexture &texture,
                             VkImageLayout old_layout, VkImageLayout new_layout,
                             VkPipelineStageFlags source_stage,
                             VkAccessFlags source_access,
                             VkPipelineStageFlags destination_stage,
                             VkAccessFlags destination_access) {
    VkImageMemoryBarrier barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    barrier.oldLayout = old_layout;
    barrier.newLayout = new_layout;
    barrier.srcAccessMask = source_access;
    barrier.dstAccessMask = destination_access;
    barrier.image = texture.image;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.layerCount = 1;
    vkCmdPipelineBarrier(command_buffer, source_stage, destination_stage, 0, 0,
                         nullptr, 0, nullptr, 1, &barrier);
  }

  void render_image(EyeSwapchain &eye, uint32_t eye_index, uint32_t image_index,
                    bool upload_test_frame, const Mat4 &mvp) {
    auto &video_texture = video_textures_.at(eye_index);
    VK_CHECK(vkWaitForFences(vk_.device, 1, &vk_.fence, VK_TRUE, UINT64_MAX));
    VK_CHECK(vkResetFences(vk_.device, 1, &vk_.fence));
    VK_CHECK(vkResetCommandBuffer(vk_.command_buffer, 0));
    VkCommandBufferBeginInfo begin{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    VK_CHECK(vkBeginCommandBuffer(vk_.command_buffer, &begin));

    if (upload_test_frame) {
      const VkImageLayout old_layout =
          video_texture.image_initialized
              ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
              : VK_IMAGE_LAYOUT_UNDEFINED;
      transition_test_image(
          vk_.command_buffer, video_texture, old_layout,
          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
          video_texture.image_initialized
              ? VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT
              : VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
          video_texture.image_initialized ? VK_ACCESS_SHADER_READ_BIT : 0,
          VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT);
      VkBufferImageCopy copy{};
      copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      copy.imageSubresource.layerCount = 1;
      copy.imageExtent = {eye_video_width_, eye_video_height_, 1};
      vkCmdCopyBufferToImage(vk_.command_buffer, video_texture.staging_buffer,
                             video_texture.image,
                             VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
      transition_test_image(
          vk_.command_buffer, video_texture,
          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
          VK_PIPELINE_STAGE_TRANSFER_BIT, VK_ACCESS_TRANSFER_WRITE_BIT,
          VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, VK_ACCESS_SHADER_READ_BIT);
      video_texture.image_initialized = true;
    }

    VkClearValue clear{};
    clear.color.float32[0] = 0.01f;
    clear.color.float32[1] = 0.01f;
    clear.color.float32[2] = 0.01f;
    // With alpha blending, this is the transparent area around the video
    // window. The test-pattern pixels themselves retain alpha = 1.
    clear.color.float32[3] = 0.0f;
    VkRenderPassBeginInfo render_pass_begin{
        VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    render_pass_begin.renderPass = render_pass_;
    render_pass_begin.framebuffer = eye.framebuffers.at(image_index);
    render_pass_begin.renderArea.extent = {width_, height_};
    render_pass_begin.clearValueCount = 1;
    render_pass_begin.pClearValues = &clear;
    vkCmdBeginRenderPass(vk_.command_buffer, &render_pass_begin,
                         VK_SUBPASS_CONTENTS_INLINE);
    if (has_test_frame_) {
      vkCmdBindPipeline(vk_.command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
                        test_graphics_pipeline_);
      vkCmdBindDescriptorSets(vk_.command_buffer,
                              VK_PIPELINE_BIND_POINT_GRAPHICS,
                              test_pipeline_layout_, 0, 1,
                              &video_texture.descriptor_set, 0, nullptr);
      vkCmdPushConstants(vk_.command_buffer, test_pipeline_layout_,
                         VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(Mat4), &mvp);
      vkCmdDraw(vk_.command_buffer, 6, 1, 0, 0);
    }
    vkCmdEndRenderPass(vk_.command_buffer);

    VK_CHECK(vkEndCommandBuffer(vk_.command_buffer));
    VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &vk_.command_buffer;
    VK_CHECK(vkQueueSubmit(vk_.queue, 1, &submit, vk_.fence));
    VK_CHECK(vkWaitForFences(vk_.device, 1, &vk_.fence, VK_TRUE, UINT64_MAX));
  }

  void sync_actions(XrTime display_time, const XrVector3f &eye_midpoint,
                    const bool eye_pose_valid) {
    XrActiveActionSet active{action_set_, XR_NULL_PATH};
    XrActionsSyncInfo sync{XR_TYPE_ACTIONS_SYNC_INFO};
    sync.countActiveActionSets = 1;
    sync.activeActionSets = &active;
    const XrResult result = xrSyncActions(session_, &sync);
    if (result == XR_SESSION_NOT_FOCUSED) {
      if (controller_callback_) {
        controller_callback_(std::array<sawOpenXR::ControllerState, 2>{});
      }

      return;
    }

    XR_CHECK(result);

    XrActionStateGetInfo get_info{XR_TYPE_ACTION_STATE_GET_INFO};
    get_info.action = quit_action_;
    XrActionStateBoolean state{XR_TYPE_ACTION_STATE_BOOLEAN};
    XR_CHECK(xrGetActionStateBoolean(session_, &get_info, &state));
    const bool pressed = state.isActive && state.currentState;
    if (pressed && !quit_was_pressed_) {
      std::cerr << "Exit requested from left Menu button.\n";
      XR_CHECK(xrRequestExitSession(session_));
    }
    quit_was_pressed_ = pressed;

    get_info.action = reset_window_action_;
    get_info.subactionPath = XR_NULL_PATH;
    XrActionStateBoolean reset_window_state{XR_TYPE_ACTION_STATE_BOOLEAN};
    XR_CHECK(xrGetActionStateBoolean(session_, &get_info,
                                     &reset_window_state));
    const bool reset_window_pressed = reset_window_state.isActive &&
                                      reset_window_state.currentState;
    if (reset_window_pressed && !reset_window_was_pressed_ &&
        eye_pose_valid) {
      grabbed_hand_ = -1;
      reset_video_window_in_front_of_head(eye_midpoint);
      std::cerr << "Video window reset in front of the user.\n";
    }
    reset_window_was_pressed_ = reset_window_pressed;

    std::array<sawOpenXR::ControllerState, 2> controller_states{};
    std::array<XrPosef, 2> grip_poses{};
    std::array<bool, 2> grip_valids{};

    for (uint32_t hand = 0; hand < hand_paths_.size(); ++hand) {
      get_info.action = grab_action_;
      get_info.subactionPath = hand_paths_[hand];
      XrActionStateBoolean grab_state{XR_TYPE_ACTION_STATE_BOOLEAN};
      XR_CHECK(xrGetActionStateBoolean(session_, &get_info, &grab_state));
      const bool grab_pressed = grab_state.isActive && grab_state.currentState;

      get_info.action = thumbstick_action_;
      get_info.subactionPath = hand_paths_[hand];
      XrActionStateVector2f thumbstick_state{XR_TYPE_ACTION_STATE_VECTOR2F};
      XR_CHECK(xrGetActionStateVector2f(session_, &get_info, &thumbstick_state));

      get_info.action = thumbstick_click_action_;
      get_info.subactionPath = hand_paths_[hand];
      XrActionStateBoolean thumbstick_click_state{XR_TYPE_ACTION_STATE_BOOLEAN};
      XR_CHECK(xrGetActionStateBoolean(session_, &get_info,
                                       &thumbstick_click_state));
      const bool thumbstick_clicked =
          thumbstick_click_state.isActive && thumbstick_click_state.currentState;

      get_info.action = front_trigger_action_;
      get_info.subactionPath = hand_paths_[hand];
      XrActionStateFloat front_trigger_state{XR_TYPE_ACTION_STATE_FLOAT};
      XR_CHECK(
          xrGetActionStateFloat(session_, &get_info, &front_trigger_state));

      XrSpaceLocation grip_location{XR_TYPE_SPACE_LOCATION};
      const XrResult location_result = xrLocateSpace(
          grip_spaces_[hand], local_space_, display_time, &grip_location);
      const bool grip_valid = location_result == XR_SUCCESS &&
                              (grip_location.locationFlags &
                               XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0 &&
                              (grip_location.locationFlags &
                               XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0;

      auto &controller_state = controller_states[hand];
      controller_state.session_focused = true;
      grip_valids[hand] = grip_valid;
      if (grip_valid) {
        grip_poses[hand] = grip_location.pose;
      }
      controller_state.thumbstick_x = thumbstick_state.isActive
                                          ? thumbstick_state.currentState.x
                                          : 0.0;
      controller_state.thumbstick_y = thumbstick_state.isActive
                                          ? thumbstick_state.currentState.y
                                          : 0.0;
      controller_state.thumbstick_click = thumbstick_clicked;
      controller_state.front_trigger_active = front_trigger_state.isActive;
      controller_state.front_trigger = std::clamp(
          static_cast<double>(front_trigger_state.currentState), 0.0, 1.0);
      controller_state.window_move_pressed = grab_pressed;
      controller_state.timestamp =
          std::chrono::duration<double>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count();

      if (grab_pressed && !grab_was_pressed_[hand] && grabbed_hand_ < 0 &&
          grip_valid && video_window_initialized_) {
        grabbed_hand_ = static_cast<int>(hand);
        grab_start_orientation_ = grip_location.pose.orientation;
        grab_start_window_orientation_ = video_window_pose_.orientation;
        grab_start_position_ = grip_location.pose.position;
        grab_start_window_position_ = video_window_pose_.position;
        std::cerr << "Video window grabbed with "
                  << (hand == 0 ? "left" : "right") << " controller.\n";
      }
      if (grabbed_hand_ == static_cast<int>(hand)) {
        if (!grab_pressed) {
          grabbed_hand_ = -1;
          std::cerr << "Video window released.\n";
        } else if (grip_valid) {
          // Translate relative to the grab point at 2x gain. Apply only
          // the controller's orientation delta, so wrist rotation turns
          // the window in place instead of orbiting it around the hand.
          constexpr float translation_gain = 2.0f;
          video_window_pose_.position = {
              grab_start_window_position_.x +
                  (grip_location.pose.position.x - grab_start_position_.x) *
                      translation_gain,
              grab_start_window_position_.y +
                  (grip_location.pose.position.y - grab_start_position_.y) *
                      translation_gain,
              grab_start_window_position_.z +
                  (grip_location.pose.position.z - grab_start_position_.z) *
                      translation_gain};
          const XrQuaternionf rotation_delta = quaternion_multiply(
              grip_location.pose.orientation,
              quaternion_conjugate(grab_start_orientation_));
          video_window_pose_.orientation = quaternion_multiply(
              rotation_delta, grab_start_window_orientation_);
        }
      }
      grab_was_pressed_[hand] = grab_pressed;
    }

    // The HRSV origin is the midpoint between the user's eyes, while the
    // virtual video plane supplies its orientation. Perform this conversion
    // after processing both grab actions so both hands use the same final
    // plane orientation, including on the A release frame.
    if (video_window_initialized_ && eye_pose_valid) {
      const XrQuaternionf plane_inverse =
          quaternion_conjugate(video_window_pose_.orientation);
      for (uint32_t hand = 0; hand < hand_paths_.size(); ++hand) {
        if (!grip_valids[hand]) {
          continue;
        }

        const XrVector3f eye_to_grip{
            grip_poses[hand].position.x - eye_midpoint.x,
            grip_poses[hand].position.y - eye_midpoint.y,
            grip_poses[hand].position.z - eye_midpoint.z};
        const XrVector3f relative_position =
            rotate_vector(plane_inverse, eye_to_grip);
        const XrQuaternionf relative_orientation = quaternion_multiply(
            plane_inverse, grip_poses[hand].orientation);

        auto &controller_state = controller_states[hand];
        controller_state.tracked = true;
        controller_state.position = {relative_position.x, relative_position.y,
                                     relative_position.z};
        controller_state.orientation = {
            relative_orientation.x, relative_orientation.y,
            relative_orientation.z, relative_orientation.w};
      }
    }

    if (controller_callback_) {
      controller_callback_(controller_states);
    }
  }

  void reset_video_window_in_front_of_head(const XrVector3f &head_position) {
    // The two eye orientations normally match; use the first eye's
    // predicted head orientation to make the window face the user.
    video_window_pose_.orientation = views_[0].pose.orientation;
    const XrVector3f forward =
        rotate_vector(video_window_pose_.orientation, {0.0f, 0.0f, -1.0f});
    constexpr float initial_distance_meters = 1.5f;
    video_window_pose_.position = {
        head_position.x + forward.x * initial_distance_meters,
        head_position.y + forward.y * initial_distance_meters,
        head_position.z + forward.z * initial_distance_meters};
    video_window_initialized_ = true;
  }

  void initialize_video_window_from_head(uint32_t view_count) {
    if (video_window_initialized_ || view_count == 0) {
      return;
    }
    XrVector3f head_position{};
    for (uint32_t i = 0; i < view_count; ++i) {
      head_position.x += views_[i].pose.position.x;
      head_position.y += views_[i].pose.position.y;
      head_position.z += views_[i].pose.position.z;
    }
    const float inverse_count = 1.0f / static_cast<float>(view_count);
    head_position.x *= inverse_count;
    head_position.y *= inverse_count;
    head_position.z *= inverse_count;
    reset_video_window_in_front_of_head(head_position);
    std::cout
        << "Placed video window 1.5 m in front of the tracked head pose.\n";
  }

  void render_loop() {
    const auto start = std::chrono::steady_clock::now();
    auto last_status = start;
    uint64_t frame_count = 0;
    uint64_t rendered_frame_count = 0;
    while (!exit_requested_) {
      poll_events();

      if (stop_requested_.load()) {
        if (session_running_ && !shutdown_exit_requested_) {
          std::cerr << "Requesting OpenXR session shutdown.\n";
          XR_CHECK(xrRequestExitSession(session_));
          shutdown_exit_requested_ = true;
          shutdown_deadline_ =
              std::chrono::steady_clock::now() + std::chrono::seconds(5);
        }

        // xrEndSession is valid only after the runtime sends STOPPING.  Keep
        // polling for that event instead of destroying a running session.
        if (!session_running_ && !shutdown_exit_requested_) {
          return;
        }
        if (shutdown_exit_requested_ &&
            std::chrono::steady_clock::now() >= shutdown_deadline_) {
          std::cerr << "Timed out waiting for OpenXR session shutdown; "
                       "destroying remaining resources.\n";
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      if (!session_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      XrFrameWaitInfo wait{XR_TYPE_FRAME_WAIT_INFO};
      XrFrameState frame_state{XR_TYPE_FRAME_STATE};
      XR_CHECK(xrWaitFrame(session_, &wait, &frame_state));

      // Locate the stereo viewer before the controllers. This makes the HRSV
      // origin and both hand poses correspond to the same predicted time.
      XrViewLocateInfo reference_locate{XR_TYPE_VIEW_LOCATE_INFO};
      reference_locate.viewConfigurationType =
          XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
      reference_locate.displayTime = frame_state.predictedDisplayTime;
      reference_locate.space = local_space_;
      XrViewState reference_view_state{XR_TYPE_VIEW_STATE};
      uint32_t reference_view_count = 0;
      XR_CHECK(xrLocateViews(session_, &reference_locate, &reference_view_state,
                             static_cast<uint32_t>(views_.size()),
                             &reference_view_count, views_.data()));
      const bool eye_pose_valid =
          reference_view_count == views_.size() &&
          (reference_view_state.viewStateFlags &
           XR_VIEW_STATE_POSITION_VALID_BIT) != 0 &&
          (reference_view_state.viewStateFlags &
           XR_VIEW_STATE_ORIENTATION_VALID_BIT) != 0;
      XrVector3f eye_midpoint{};
      if (eye_pose_valid) {
        for (uint32_t i = 0; i < reference_view_count; ++i) {
          eye_midpoint.x += views_[i].pose.position.x;
          eye_midpoint.y += views_[i].pose.position.y;
          eye_midpoint.z += views_[i].pose.position.z;
        }
        const float inverse_count =
            1.0f / static_cast<float>(reference_view_count);
        eye_midpoint.x *= inverse_count;
        eye_midpoint.y *= inverse_count;
        eye_midpoint.z *= inverse_count;
        initialize_video_window_from_head(reference_view_count);
      }
      sync_actions(frame_state.predictedDisplayTime, eye_midpoint,
                   eye_pose_valid);
      XR_CHECK(xrBeginFrame(session_, nullptr));

      std::vector<XrCompositionLayerBaseHeader *> layers;
      XrCompositionLayerProjection projection{
          XR_TYPE_COMPOSITION_LAYER_PROJECTION};
      std::vector<XrCompositionLayerProjectionView> projection_views;
      if (frame_state.shouldRender) {
        ++rendered_frame_count;
        XrViewLocateInfo locate{XR_TYPE_VIEW_LOCATE_INFO};
        locate.viewConfigurationType =
            XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
        locate.displayTime = frame_state.predictedDisplayTime;
        locate.space = local_space_;
        XrViewState view_state{XR_TYPE_VIEW_STATE};
        uint32_t view_count = 0;
        XR_CHECK(xrLocateViews(session_, &locate, &view_state,
                               static_cast<uint32_t>(views_.size()),
                               &view_count, views_.data()));
        if (view_count == views_.size()) {
          initialize_video_window_from_head(view_count);
          static bool logged_first_frame = false;
          if (!logged_first_frame) {
            std::cerr << "Rendering stereo frames: " << width_ << "x" << height_
                      << " per eye.\n";
            logged_first_frame = true;
          }
          const bool got_test_frame = pull_test_frame();
          projection_views.assign(view_count,
                                  {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW});
          for (uint32_t i = 0; i < view_count; ++i) {
            auto &eye = eye_swapchains_[i];
            uint32_t image_index = 0;
            XrSwapchainImageAcquireInfo acquire{
                XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
            XR_CHECK(
                xrAcquireSwapchainImage(eye.handle, &acquire, &image_index));
            XrSwapchainImageWaitInfo image_wait{
                XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
            image_wait.timeout = XR_INFINITE_DURATION;
            XR_CHECK(xrWaitSwapchainImage(eye.handle, &image_wait));
            // Mono frames are duplicated; side-by-side frames contribute the
            // corresponding half to each eye.
            const Mat4 mvp =
                multiply(projection_matrix(views_[i].fov),
                         multiply(inverse_rigid_matrix(views_[i].pose),
                                  pose_matrix(video_window_pose_)));
            render_image(eye, i, image_index, got_test_frame, mvp);
            XrSwapchainImageReleaseInfo release{
                XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
            XR_CHECK(xrReleaseSwapchainImage(eye.handle, &release));
            projection_views[i].pose = views_[i].pose;
            projection_views[i].fov = views_[i].fov;
            projection_views[i].subImage.swapchain = eye.handle;
            projection_views[i].subImage.imageRect.extent = {
                static_cast<int32_t>(width_), static_cast<int32_t>(height_)};
          }
          projection.space = local_space_;
          if (environment_blend_mode_ ==
              XR_ENVIRONMENT_BLEND_MODE_ALPHA_BLEND) {
            // The swapchain's clear area has alpha 0 and the video
            // pixels have alpha 1. Tell the compositor to preserve
            // that alpha when it blends this layer over passthrough.
            projection.layerFlags =
                XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
                XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
          }
          projection.viewCount = view_count;
          projection.views = projection_views.data();
          layers.push_back(
              reinterpret_cast<XrCompositionLayerBaseHeader *>(&projection));
        }
      }
      XrFrameEndInfo end{XR_TYPE_FRAME_END_INFO};
      end.displayTime = frame_state.predictedDisplayTime;
      end.environmentBlendMode = environment_blend_mode_;
      end.layerCount = static_cast<uint32_t>(layers.size());
      end.layers = layers.data();
      XR_CHECK(xrEndFrame(session_, &end));

      ++frame_count;
      const auto now = std::chrono::steady_clock::now();
      if (now - last_status >= std::chrono::seconds(1)) {
        std::ostringstream heartbeat;
        heartbeat << "XR loop heartbeat: " << frame_count << " frames, "
                  << rendered_frame_count << " rendered; GStreamer frames="
                  << test_frame_count_ << "; shouldRender="
                  << (frame_state.shouldRender ? "true" : "false");

        if (latest_video_age_valid_) {
          heartbeat << "; latest-video-age=" << latest_video_age_ms_ << " ms";
        }

        if (sender_video_age_valid_) {
          heartbeat << "; sender-video-age=" << sender_video_age_ms_ << " ms";
        }

        if (video_queue_ != nullptr) {
          guint queue_buffers = 0;
          guint64 queue_time = 0;

          g_object_get(video_queue_, "current-level-buffers", &queue_buffers,
                       "current-level-time", &queue_time, nullptr);
          heartbeat << "; decoder-queue=" << queue_buffers << " buffers/"
                    << static_cast<double>(queue_time) /
                           static_cast<double>(GST_MSECOND) << " ms";
        }

        GstStructure *sink_stats = nullptr;
        if (test_sink_ != nullptr) {
          g_object_get(test_sink_, "stats", &sink_stats, nullptr);
        }

        if (sink_stats != nullptr) {
          guint64 dropped = 0;
          gst_structure_get_uint64(sink_stats, "dropped", &dropped);
          heartbeat << "; appsink-dropped=" << dropped;
          gst_structure_free(sink_stats);
        }

        std::cout << heartbeat.str();
        last_status = now;
      }
    }
  }
};

} // namespace

sawOpenXR::OpenXRRuntime::OpenXRRuntime(const std::string &video_pipeline,
                                        VideoType video_type,
                                        ControllerCallback controller_callback,
                                        ErrorCallback error_callback,
                                        GStreamerCallback gstreamer_status_callback,
                                        GStreamerCallback gstreamer_warning_callback)
    : m_video_pipeline(video_pipeline), m_video_type(video_type),
      m_controller_callback(std::move(controller_callback)),
      m_error_callback(std::move(error_callback)),
      m_gstreamer_status_callback(std::move(gstreamer_status_callback)),
      m_gstreamer_warning_callback(std::move(gstreamer_warning_callback)) {}

void sawOpenXR::OpenXRRuntime::Run(void) {
  try {
    App app(m_video_pipeline, m_video_type, m_controller_callback,
            m_gstreamer_status_callback, m_gstreamer_warning_callback,
            m_stop_requested);

    app.run();
  } catch (const std::exception &error) {
    if (m_error_callback) {
      m_error_callback(error.what());
    }
  }
}

void sawOpenXR::OpenXRRuntime::RequestStop(void) {
  m_stop_requested.store(true);
}
