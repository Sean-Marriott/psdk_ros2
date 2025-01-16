/*
* TODO File description
*/ 

#include "psdk_wrapper/modules/video_stream.hpp"

namespace psdk_ros2
{
VideoStreamModule::VideoStreamModule(const std::string &name)
    : rclcpp_lifecycle::LifecycleNode(
        name, "",
        rclcpp::NodeOptions().arguments(
            {"--ros-args", "-r",
            name + ":" + std::string("__node:=") + name}))
{
    RCLCPP_INFO(get_logger(), "Creating VideoStreamModule");
}

VideoStreamModule::~VideoStreamModule()
{
    RCLCPP_INFO(get_logger(), "Destroying VideoStreamModule");
}

VideoStreamModule::CallbackReturn 
VideoStreamModule::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Configuring VideoStreamModule");
    video_stream_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "psdk_image_bridge/input_image", 10,
        std::bind(&VideoStreamModule::videoStreamCallback, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
}

VideoStreamModule::CallbackReturn
VideoStreamModule::on_activate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Activating VideoStreamModule");
    return CallbackReturn::SUCCESS;
}

VideoStreamModule::CallbackReturn
VideoStreamModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Deactivating VideoStreamModule");
    return CallbackReturn::SUCCESS;
}

VideoStreamModule::CallbackReturn
VideoStreamModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Cleaning up VideoStreamModule");
    video_stream_sub_.reset();
    return CallbackReturn::SUCCESS;
}

VideoStreamModule::CallbackReturn
VideoStreamModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "Shutting down VideoStreamModule");
    return CallbackReturn::SUCCESS;
}

bool
VideoStreamModule::init()
{
    if (is_module_initialized_)
    {
        RCLCPP_WARN(get_logger(),
                    "VideoStreamModule is already initialized, skipping.");
        return true;
    }

    RCLCPP_INFO(get_logger(), "Initiating video stream module");
    // TODO: Initialisation of ffmpeg codec and context
    // TODO: Init of DJI highspeed data channel?
    is_module_initialized_ = true;
    return true;
}

bool
VideoStreamModule::deinit()
{
    RCLCPP_INFO(get_logger(), "Deinitializing video stream module");
    // TODO
    is_module_initialized_ = false;
    return true;
}

void
VideoStreamModule::videoStreamCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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