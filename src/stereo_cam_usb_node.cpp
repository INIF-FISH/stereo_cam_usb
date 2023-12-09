#include "../include/stereo_cam_usb_node.h"

namespace stereo_cam_usb
{
    StereoCamUsbNode::StereoCamUsbNode(const rclcpp::NodeOptions &options)
        : Node("stereo_cam_usb", options)
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("stereo_cam_usb");

        rclcpp::QoS qos(1);
        qos.reliable();

        this->camera_index = this->declare_parameter<int>("camera_index", 0);
        this->camera_params_path = this->declare_parameter<std::string>("camera_params_path", package_share_directory + "/params/camera.yaml");
        this->camera_width = this->declare_parameter<int>("camera_width", 2560);
        this->camera_height = this->declare_parameter<int>("camera_height", 720);
        this->camera_fps = this->declare_parameter<int>("camera_fps", 60);
        this->camera_exposure = this->declare_parameter<int>("camera_exposure", 50);
        this->_cameraPubThread = std::thread(std::bind(&StereoCamUsbNode::readImageAndPub, this));
        this->timer_ = this->create_wall_timer(100ms, std::bind(&StereoCamUsbNode::respond, this));
        this->_image_pub = std::make_shared<image_transport::Publisher>(image_transport::create_publisher(this, "stereo_cam_usb/image_raw", qos.get_rmw_qos_profile()));
    }

    StereoCamUsbNode::~StereoCamUsbNode()
    {
    }

    bool StereoCamUsbNode::init()
    {
        if (camera_index == -1 || camera_params_path.empty())
            return false;
        this->_StereoCamUsb = std::make_shared<StereoCamUsb>(StereoCamUsb(this->camera_index));
        if (!this->_StereoCamUsb->is_open())
            return false;
        this->_StereoCamUsb->setCAP_PROP_FRAME_WIDTH(this->camera_width);
        this->_StereoCamUsb->setCAP_PROP_FRAME_HEIGHT(this->camera_height);
        this->_StereoCamUsb->setCAP_PROP_FPS(this->camera_fps);
        this->_StereoCamUsb->setCAP_PROP_EXPOSURE(this->camera_exposure);
        if (!this->_StereoCamUsb->read_params(this->camera_params_path))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read params for camera index %d !", this->camera_index);
            this->_StereoCamUsb->release();
            return false;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Success to read params for camera index %d !", this->camera_index);
            return true;
        }
    }

    void StereoCamUsbNode::readImageAndPub()
    {
        while (this->_node_alive_flag)
        {
            while (!this->_camera_alive_flag)
            {
                RCLCPP_WARN(this->get_logger(), "Wait for camera index %d open ...", this->camera_index);
                sleep(1);
            }
            RCLCPP_INFO(this->get_logger(), "Success to open camera index %d !", this->camera_index);
            while (this->_camera_alive_flag)
            {
                cv::Mat image;
                this->_StereoCamUsb->readImage();
                this->_StereoCamUsb->correctImage();
                // this->_StereoCamUsb->processSGBM();
                this->_StereoCamUsb->getImage(image);
                if (image.empty())
                    continue;
                auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", image).toImageMsg();
                this->_image_pub->publish(image_msg);
            }
        }
    }

    void StereoCamUsbNode::respond()
    {
        this->get_parameter("camera_index", this->camera_index);
        this->get_parameter("camera_params_path", this->camera_params_path);
        this->get_parameter("camera_width", this->camera_width);
        this->get_parameter("camera_height", this->camera_height);
        this->get_parameter("camera_fps", this->camera_fps);
        this->get_parameter("camera_exposure", this->camera_exposure);
        bool _param_change_flag = false;
        if (this->_camera_alive_flag)
        {
            if (this->_StereoCamUsb->get_frame_width() != this->camera_width)
            {
                RCLCPP_INFO(this->get_logger(), "Set FRAME_WIDTH to %d !", this->camera_width);
                _param_change_flag = true;
            }
            if (this->_StereoCamUsb->get_frame_height() != this->camera_height)
            {
                RCLCPP_INFO(this->get_logger(), "Set FRAME_HEIGHT to %d !", this->camera_height);
                _param_change_flag = true;
            }
            if (this->_StereoCamUsb->get_frame_fps() != this->camera_fps)
            {
                RCLCPP_INFO(this->get_logger(), "Set CAP_FPS to %d !", this->camera_fps);
                _param_change_flag = true;
            }
            if(this->_StereoCamUsb->get_frame_exposure() != this->camera_exposure)
            {
                RCLCPP_INFO(this->get_logger(), "Set CAP_PROP_EXPOSURE to %d !", this->camera_exposure);
                _param_change_flag = true;
            }
            if (_param_change_flag)
            {
                RCLCPP_INFO(this->get_logger(), "Camera params changed ! Reopen camera index %d !", this->camera_index);
                this->_camera_alive_flag = false;
                while (this->_StereoCamUsb->is_open())
                {
                    sleep(1);
                    this->_StereoCamUsb->release();
                    RCLCPP_WARN(this->get_logger(), "Wait for camera index %d release ...", this->camera_index);
                }
            }
        }
        if (!this->_camera_alive_flag)
            this->_camera_alive_flag = this->init();
    }

    void StereoCamUsbNode::stop()
    {
        this->timer_->cancel();
        this->_camera_alive_flag = false;
        this->_StereoCamUsb->release();
        RCLCPP_INFO(this->get_logger(), "Release camera index %d !", this->camera_index);
    }

} // namespace stereo_cam_usb

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<stereo_cam_usb::StereoCamUsbNode>());
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stereo_cam_usb::StereoCamUsbNode)