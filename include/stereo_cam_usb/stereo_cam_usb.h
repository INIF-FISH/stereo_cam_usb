#ifndef _STEREO_CAM_USB_H
#define _STEREO_CAM_USB_H

// C++
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <atomic>
#include <chrono>
#include <sstream>
#include <fstream>
#include <unistd.h>

// opencv
#include <opencv2/opencv.hpp>

namespace stereo_cam_usb
{
    using namespace std::chrono;
    struct CameraParameters
    {
        cv::Mat camera_matrix_left;
        cv::Mat camera_matrix_right;
        cv::Mat distortion_coefficients_left;
        cv::Mat distortion_coefficients_right;
        cv::Mat translation_of_camera_right;
        cv::Mat rotation_of_camera_right;
        double baseline;

        cv::Mat Rl, Rr, Pl, Pr, Q;

        CameraParameters(){};
        CameraParameters(
            cv::Mat camera_matrix_left,
            cv::Mat camera_matrix_right,
            cv::Mat distortion_coefficients_left,
            cv::Mat distortion_coefficients_right,
            cv::Mat translation_of_camera_right,
            cv::Mat rotation_of_camera_right)
        {
            this->distortion_coefficients_right = distortion_coefficients_right;
            this->distortion_coefficients_left = distortion_coefficients_left;
            this->camera_matrix_left = camera_matrix_left;
            this->camera_matrix_right = camera_matrix_right;
            this->translation_of_camera_right = translation_of_camera_right;
            this->rotation_of_camera_right = rotation_of_camera_right;
        };
    };

    class StereoCamUsb
    {
    private:
        std::shared_ptr<cv::VideoCapture> cap;
        CameraParameters params;
        cv::Mat frame;

        int _frame_width = 0;
        int _frame_height = 0;
        int _frame_exposure = 0;
        int _frame_fps = 0;

        bool _is_calcTransform_flag = false;
        cv::Mat undistmap1l, undistmap2l, undistmap1r, undistmap2r;

    public:
        bool _ifShowErr = false;

    public:
        StereoCamUsb(const int index);
        ~StereoCamUsb();

        bool read_params(const std::string &path);
        bool is_open();
        void readImage();
        void calcTransform();
        void correctImage();
        void processSGBM();
        void getImage(cv::Mat &img);

        int getCAP_PROP_FRAME_WIDTH();
        int getCAP_PROP_FRAME_HEIGHT();
        int getCAP_PROP_AUTO_EXPOSURE();
        int getCAP_PROP_EXPOSURE();
        int getCAP_PROP_FOURCC();
        int getCAP_PROP_FPS();

        void setCAP_PROP_FRAME_WIDTH(const int value);
        void setCAP_PROP_FRAME_HEIGHT(const int value);
        void setCAP_PROP_AUTO_EXPOSURE(const int value);
        void setCAP_PROP_EXPOSURE(const int value);
        void setCAP_PROP_FOURCC(char c1, char c2, char c3, char c4);
        void setCAP_PROP_FPS(const int value);

        int get_frame_width();
        int get_frame_height();
        int get_frame_exposure();
        int get_frame_fps();

        void release();
    };
} // namespace stereo_cam_usb

#endif // _STEREO_CAM_USB_H