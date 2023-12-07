#ifndef _STEREO_CAM_USB_NODE_H
#define _STEREO_CAM_USB_NODE_H

#include "./stereo_cam_usb/stereo_cam_usb.h"

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace stereo_cam_usb
{
    class StereoCamUsbNode : public rclcpp::Node
    {
    private:
        std::shared_ptr<StereoCamUsb> _StereoCamUsb;
        std::thread _cameraPubThread;
        std::shared_ptr<rclcpp::WallTimer<std::_Bind<void (stereo_cam_usb::StereoCamUsbNode::*(stereo_cam_usb::StereoCamUsbNode *))()>, (void *)nullptr>> timer_;
        std::shared_ptr<image_transport::Publisher> _image_pub;

        bool _node_alive_flag = true;
        bool _camera_alive_flag = false;

        int camera_index = -1;
        std::string camera_params_path;
        int camera_width = 2560;
        int camera_height = 720;
        int camera_fps = 60;
        int camera_exposure = 100;

    public:
        StereoCamUsbNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~StereoCamUsbNode();

        bool init();
        void readImageAndPub();
        void respond();
        void stop();
    };
} // namespace stereo_cam_usb

#endif // _STEREO_CAM_USB_NODE_H