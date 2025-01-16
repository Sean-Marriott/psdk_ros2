/*
 * TODO: Add file description
 */

#ifndef PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_USB_CAMERA_HPP_
#define PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_USB_CAMERA_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

namespace psdk_ros2
{

class UsbCameraModule : public rclcpp_lifecycle::LifecycleNode
{
  public:
  /**
   * @brief Construct a new UsbCameraModule object
   * @param node_name Name of the node
   */
  explicit UsbCameraModule(const std::string& name);

  /**
   * @brief Destroy the usb camera module object
   */
  ~UsbCameraModule();

  /**
   * @brief Configure the usb camera module. Creates the ROS 2 subscribers
   * and services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state);

  /**
   * @brief Activate the usb camera module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Clean the usb camera module. Resets the ROS 2 subscribers and
   * services.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state);

  /**
   * @brief Deactivate the usb camera module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state);

  /**
   * @brief Shutdown the usb camera module.
   * @param state rclcpp_lifecycle::State. Current state of the node.
   * @return CallbackReturn SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state);

  /**
   * @brief Initialize the usb camera module.
   * @return true/false
   */
  bool init();

  /**
   * @brief Deinitialize the usb camera module
   * @return true/false
   */
  bool deinit();

  private:
  /**
   * @brief Callback function for the usb camera
   * @param msg Image message
   */
  void usbCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr usb_camera_sub_;

  bool is_module_initialized_{false};

  cv_bridge::CvImagePtr cv_ptr_;
  
};
} // namespace psdk_ros2

#endif // PSDK_WRAPPER_INCLUDE_PSDK_WRAPPER_MODULES_USB_CAMERA_HPP_