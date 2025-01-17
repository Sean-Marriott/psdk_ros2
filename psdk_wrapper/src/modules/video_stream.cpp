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
    (void)state;
    RCLCPP_INFO(get_logger(), "Configuring VideoStreamModule");
    video_stream_sub_ = create_subscription<sensor_msgs::msg::Image>(
        "psdk_image_bridge/input_image", 10,
        std::bind(&VideoStreamModule::videoStreamCallback, this, std::placeholders::_1));
    return CallbackReturn::SUCCESS;
}

VideoStreamModule::CallbackReturn
VideoStreamModule::on_activate(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(get_logger(), "Activating VideoStreamModule");
    return CallbackReturn::SUCCESS;
}

VideoStreamModule::CallbackReturn
VideoStreamModule::on_deactivate(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(get_logger(), "Deactivating VideoStreamModule");
    return CallbackReturn::SUCCESS;
}

VideoStreamModule::CallbackReturn
VideoStreamModule::on_cleanup(const rclcpp_lifecycle::State &state)
{
    (void)state;
    RCLCPP_INFO(get_logger(), "Cleaning up VideoStreamModule");
    video_stream_sub_.reset();
    return CallbackReturn::SUCCESS;
}

VideoStreamModule::CallbackReturn
VideoStreamModule::on_shutdown(const rclcpp_lifecycle::State &state)
{
    (void)state;
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
    // TODO: Init of DJI highspeed data channel?

    RCLCPP_INFO(get_logger(), "Initiating video stream module");

	avdevice_register_all();
    const AVCodec *codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec)
    {
        RCLCPP_ERROR(get_logger(), "Error finding codec");
        return false;
    }
    RCLCPP_INFO(get_logger(), "Found codec: %s", codec->name);

    codecContext = avcodec_alloc_context3(codec);
    if (!codecContext)
    {
        RCLCPP_ERROR(get_logger(), "Error allocating codec context");
        return false;
    }
    RCLCPP_INFO(get_logger(), "Allocated codec context");

    // Set codec parameters
	codecContext->width 		= WIDTH;
	codecContext->height		= HEIGHT;
	codecContext->bit_rate		= BITRATE;
	codecContext->time_base		= {1, FPS};
	codecContext->framerate		= {FPS, 1};
	codecContext->gop_size		= 2;
	codecContext->max_b_frames	= 0;
	codecContext->pix_fmt 		= AV_PIX_FMT_YUV420P;

    // Set codec options
    int result;
    // TODO: Is this correct?
    result = av_opt_set(codecContext, "threads", "2", 0);
    RCLCPP_INFO(get_logger(), "Set codec option threads: %d", result);
    result = av_opt_set(codecContext->priv_data, "preset", "veryfast", 0);
    RCLCPP_INFO(get_logger(), "Set codec option preset: %d", result);
    result = av_opt_set(codecContext->priv_data, "tune", "zerolatency", 0);
    RCLCPP_INFO(get_logger(), "Set codec option tune: %d", result);

    // Open codec
    if (avcodec_open2(codecContext, codec, nullptr) < 0)
    {
        RCLCPP_ERROR(get_logger(), "Error opening codec");
        return false;
    }
    RCLCPP_INFO(get_logger(), "Opened codec");

    // Allocate buffer for AVFrame
    result = av_frame_get_buffer(avFrame, 0);
    if (result < 0)
    {
        RCLCPP_ERROR(get_logger(), "Error allocating buffer for AVFrame");
        return false;
    }

    last_status_time = std::chrono::steady_clock::now();

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
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    frame = cv_ptr_ -> image;

    loop_start_time = std::chrono::steady_clock::now();

    // Allocate memory for the raw frame
    std::vector<uint8_t> raw_frame(WIDTH * HEIGHT * 3 / 2);

    cv::cvtColor(frame, frame, cv::COLOR_BGR2YUV_I420);
	raw_frame.assign(frame.begin<uint8_t>(), frame.end<uint8_t>());
    
    // Copy data from raw_frame to AVFrame
	for (int i = 0; i < codecContext->height; ++i)
    {
		memcpy(avFrame->data[0] + i * avFrame->linesize[0], raw_frame.data() + i * WIDTH, codecContext->width);
    }

	for (int i = 0; i < codecContext -> height / 2; ++i)
	{
		memcpy(avFrame->data[1] + i * avFrame->linesize[1], raw_frame.data() + WIDTH * HEIGHT + i * WIDTH / 2, codecContext->width / 2);
		memcpy(avFrame->data[2] + i * avFrame->linesize[2], raw_frame.data() + WIDTH * HEIGHT * 5 / 4 + i * WIDTH / 2, codecContext->width / 2);
	}

    int ret = avcodec_send_frame(codecContext, avFrame);
    if (ret < 0)
    {
        RCLCPP_ERROR(get_logger(), "Error sending frame to codec context");
        return;
    }

    // Encode packet
    AVPacket *packet = av_packet_alloc();
    if (!packet)
    {
        RCLCPP_ERROR(get_logger(), "Error allocating video packet");
        return;
    }

    av_init_packet(packet);
    packet->data = nullptr;
    packet->size = 0;

    int result;
    result = avcodec_receive_packet(codecContext, packet);
    if (result == AVERROR(EAGAIN) || result == AVERROR_EOF)
    {
        RCLCPP_ERROR(get_logger(), "Error encoding video frame");
        av_packet_free(&packet);
        return;
    }
    else if (result < 0)
    {
        RCLCPP_ERROR(get_logger(), "Error encoding video frame");
        av_packet_free(&packet);
        return;
    }
    else if (result == 0)
    {
        // Process the packet (copy the data out)
        // Allocate memory for the encoded data
        uint8_t *encoded_data = new uint8_t[packet->size + VIDEO_FRAME_AUD_LEN];
        if (!encoded_data)
        {
            RCLCPP_ERROR(get_logger(), "Error allocating memory for encoded data");
            av_packet_free(&packet);
            return;
        }

        // Copy the packet data into the allocated memory
        memcpy(encoded_data, packet->data, packet->size);
        // Copy the AUD info to the end of the buffer
        memcpy(&encoded_data[packet->size], s_frameAudInfo, VIDEO_FRAME_AUD_LEN);

        if (psdk_connector_running_)
        {
            T_DjiReturnCode returnCode;
            int sent_data_size = 0;
            while (packet->size + VIDEO_FRAME_AUD_LEN - sent_data_size)
            {
                int sending_data_size = packet->size + VIDEO_FRAME_AUD_LEN - sent_data_size;
                if (sending_data_size > MAX_VIDEO_FRAME_SIZE)
                {
                    sending_data_size = MAX_VIDEO_FRAME_SIZE;
                }

                returnCode = DjiPayloadCamera_SendVideoStream((const uint8_t*) encoded_data + sent_data_size, sending_data_size);
                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    RCLCPP_ERROR(get_logger(), "Error sending video stream data: 0x%08lX", returnCode);
                }
                sent_data_size += sending_data_size;
            }

            time_since_last_status = std::chrono::steady_clock::now() - last_status_time;
            if (time_since_last_status > std::chrono::duration<double>(1))
            {
                T_DjiDataChannelState videoStreamState = {0};
                returnCode = DjiPayloadCamera_GetVideoStreamState(&videoStreamState);

                if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS)
                {
                    RCLCPP_ERROR(get_logger(), "Error getting video stream state: 0x%08lX", returnCode);
                }
                else
                {
                RCLCPP_INFO(
                    get_logger(),
                    "video stream state: realtimeBandwidthLimit: %d, realtimeBandwidthBeforeFlowController: %d, realtimeBandwidthAfterFlowController:%d busyState: %d.",
                    videoStreamState.realtimeBandwidthLimit, videoStreamState.realtimeBandwidthBeforeFlowController,
                    videoStreamState.realtimeBandwidthAfterFlowController,
                    videoStreamState.busyState);
                    last_status_time = std::chrono::steady_clock::now();
                    // TODO Confirm equivalent to psdk_image_bridge repo logging
                }
                last_status_time = std::chrono::steady_clock::now();
            }
        } 

        // Free the encoded data
        delete[] encoded_data;   
    }

    // Free the packet
    av_packet_free(&packet);
}


} // namespace psdk_ros2