/*
* TODO File description
*/ 

#include "psdk_wrapper/modules/usb_camera.hpp"

namespace psdk_ros2
{
UsbCameraModule::UsbCameraModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
        name, "",
        rclcpp::NodeOptions().arguments(
            {"--ros-args", "-r",
            name + ":" + std::string("__node:=") + name}))
{
    RCLCPP_INFO(get_logger(), "Creating UsbCameraModule");
}

UsbCameraModule::~UsbCameraModule()
{
    RCLCPP_INFO(get_logger(), "Destroying UsbCameraModule");
}

UsbCameraModule::CallbackReturn 
UsbCameraModule::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Configuring UsbCameraModule");
    usb_camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "psdk_image_bridge/input_image", 10,
        std::bind(&UsbCameraModule::usbCameraCallback, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
}

UsbCameraModule::CallbackReturn
UsbCameraModule::on_activate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Activating UsbCameraModule");
    return CallbackReturn::SUCCESS;
}

UsbCameraModule::CallbackReturn
UsbCameraModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Deactivating UsbCameraModule");
    return CallbackReturn::SUCCESS;
}

UsbCameraModule::CallbackReturn
UsbCameraModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Cleaning up UsbCameraModule");
    usb_camera_sub_.reset();
    return CallbackReturn::SUCCESS;
}

UsbCameraModule::CallbackReturn
UsbCameraModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down UsbCameraModule");
    return CallbackReturn::SUCCESS;
}

bool
UsbCameraModule::init()
{
    if (is_module_initialized_)
    {
        RCLCPP_WARN(get_logger(),
                    "UsbCameraModule is already initialized, skipping.");
        return true;
    }

    RCLCPP_INFO(get_logger(), "Initiating usb camera manager");
    // TODO: Initialisation of ffmpeg codec and context
    // TODO: Init of DJI highspeed data channel?
    is_module_initialized_ = true;
    return true;
}

bool
UsbCameraModule::deinit()
{
    RCLCPP_INFO(get_logger(), "Deinitializing usb camera manager");
    // TODO
    is_module_initialized_ = false;
    return true;
}

void
UsbCameraModule::usbCameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try 
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
}


} // namespace psdk_ros2