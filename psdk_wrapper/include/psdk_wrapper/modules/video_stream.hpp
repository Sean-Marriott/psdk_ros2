/*
 * TODO: Add file description
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_VIDEO_STREAM_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_VIDEO_STREAM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

namespace psdk_ros2
{

class VideoStreamModule : public rclcpp_lifecycle::LifecycleNode
{
  public:
  /**
   * @brief Construct a new VideoStreamModule object
   * @param node_name Name of the node
   */
  explicit VideoStreamModule(const std::string& name);

  /**
   * @brief Destroy the video stream module object
   */
  ~VideoStreamModule();

  /**
   * @brief Configure the video stream module. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activate the video stream module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Clean the video stream module. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);

  /**
   * @brief Deactivate the video stream module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */Ca
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Shutdown the video stream module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the video stream module.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the video stream module
   * @return true/false
   */
  bool deinit();

  private:
  /**
   * @brief Callback function for the video stream
   * @param msg Image message
   */
  void videoStreamCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_stream_sub_;

  bool is_module_initialized_{false};

  cv_bridge::CvImagePtr cv_ptr_;
  
};
} // namespace psdk_ros2

#endif // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_VIDEO_STREAM_HPP_