#include "../../include/stereo_cam_usb/stereo_cam_usb.h"

namespace stereo_cam_usb
{
    StereoCamUsb::StereoCamUsb(const int index)
    {
        this->cap = std::make_shared<cv::VideoCapture>(cv::VideoCapture(index, cv::CAP_V4L2));
        this->_frame_width = this->getCAP_PROP_FRAME_WIDTH();
        this->_frame_height = this->getCAP_PROP_FRAME_HEIGHT();
        this->_frame_fps = this->getCAP_PROP_FPS();
        this->_frame_exposure = this->getCAP_PROP_EXPOSURE();
        this->setCAP_PROP_AUTO_EXPOSURE(1);
    }

    StereoCamUsb::~StereoCamUsb()
    {
    }

    bool StereoCamUsb::read_params(const std::string &path)
    {
        if (access(path.c_str(), F_OK) != 0)
            return false;
        cv::FileStorage fs;
        try
        {
            fs = cv::FileStorage(path, cv::FileStorage::READ);
        }
        catch (const std::exception &e)
        {
            if (_ifShowErr)
                std::cerr << e.what() << '\n';
            return false;
        }
        fs["camera_matrix_left"] >> this->params.camera_matrix_left;
        fs["camera_matrix_right"] >> this->params.camera_matrix_right;
        fs["distortion_coefficients_left"] >> this->params.distortion_coefficients_left;
        fs["distortion_coefficients_right"] >> this->params.distortion_coefficients_right;
        fs["translation_of_camera_right"] >> this->params.translation_of_camera_right;
        fs["rotation_of_camera_right"] >> this->params.rotation_of_camera_right;
        return true;
    }

    void StereoCamUsb::readImage()
    {
        assert(this->cap != nullptr);
        this->cap->read(this->frame);
    }

    bool StereoCamUsb::is_open()
    {
        return this->cap->isOpened();
    }

    void StereoCamUsb::calcTransform()
    {
        cv::stereoRectify(this->params.camera_matrix_left, this->params.distortion_coefficients_left, this->params.camera_matrix_right, this->params.distortion_coefficients_right,
                          cv::Size(this->_frame_width / 2, this->_frame_height), this->params.rotation_of_camera_right, this->params.translation_of_camera_right, this->params.Rl, this->params.Rr, this->params.Pl, this->params.Pr,
                          this->params.Q, cv::CALIB_ZERO_DISPARITY, 0);
        initUndistortRectifyMap(this->params.camera_matrix_left, this->params.distortion_coefficients_left, this->params.Rl, this->params.Pl, cv::Size(this->_frame_width / 2, this->_frame_height), CV_16SC2, undistmap1l, undistmap2l);
        initUndistortRectifyMap(this->params.camera_matrix_right, this->params.distortion_coefficients_right, this->params.Rr, this->params.Pr, cv::Size(this->_frame_width / 2, this->_frame_height), CV_16SC2, undistmap1r, undistmap2r);
        this->_is_calcTransform_flag = true;
    }

    void StereoCamUsb::correctImage()
    {
        if (this->frame.empty())
            return;
        if (!this->_is_calcTransform_flag)
            this->calcTransform();
        cv::cvtColor(this->frame, this->frame, cv::COLOR_BGR2GRAY);
        cv::Mat image_l, image_r;
        this->frame(cv::Rect(0, 0, this->frame.cols / 2, this->frame.rows)).copyTo(image_l);
        this->frame(cv::Rect(this->frame.cols / 2, 0, this->frame.cols / 2, this->frame.rows)).copyTo(image_r);
        remap(image_l, image_l, this->undistmap1l, this->undistmap2l, cv::INTER_LINEAR);
        remap(image_r, image_r, this->undistmap1r, this->undistmap2r, cv::INTER_LINEAR);
        image_l.copyTo(this->frame(cv::Rect(0, 0, this->frame.cols / 2, this->frame.rows)));
        image_r.copyTo(this->frame(cv::Rect(this->frame.cols / 2, 0, this->frame.cols / 2, this->frame.rows)));
        this->params.rotation_of_camera_right = this->params.Rr * this->params.rotation_of_camera_right * this->params.Rl.t();
        this->params.translation_of_camera_right = this->params.Rr * this->params.translation_of_camera_right;
        this->params.baseline = this->params.translation_of_camera_right.at<double>(cv::Point2i(0, 0));
    }

    void StereoCamUsb::processSGBM()
    {
        cv::Mat disparity_sgbm;
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
            0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);
        sgbm->compute(this->frame(cv::Rect(0, 0, this->frame.cols / 2, this->frame.rows)), this->frame(cv::Rect(this->frame.cols / 2, 0, this->frame.cols / 2, this->frame.rows)), disparity_sgbm);
        disparity_sgbm.convertTo(frame, CV_32F, 1.0 / 16.0f);
    }

    void StereoCamUsb::getImage(cv::Mat &img)
    {
        img = this->frame.clone();
    }

    int StereoCamUsb::getCAP_PROP_FRAME_WIDTH()
    {
        assert(this->cap != nullptr);
        return this->cap->get(cv::CAP_PROP_FRAME_WIDTH);
    }

    int StereoCamUsb::getCAP_PROP_FRAME_HEIGHT()
    {
        assert(this->cap != nullptr);
        return this->cap->get(cv::CAP_PROP_FRAME_HEIGHT);
    }

    int StereoCamUsb::getCAP_PROP_AUTO_EXPOSURE()
    {
        assert(this->cap != nullptr);
        return this->cap->get(cv::CAP_PROP_AUTO_EXPOSURE);
    }

    int StereoCamUsb::getCAP_PROP_EXPOSURE()
    {
        assert(this->cap != nullptr);
        return this->cap->get(cv::CAP_PROP_EXPOSURE);
    }

    int StereoCamUsb::getCAP_PROP_FOURCC()
    {
        assert(this->cap != nullptr);
        return this->cap->get(cv::CAP_PROP_FOURCC);
    }

    int StereoCamUsb::getCAP_PROP_FPS()
    {
        assert(this->cap != nullptr);
        return this->cap->get(cv::CAP_PROP_FPS);
    }

    void StereoCamUsb::setCAP_PROP_FRAME_WIDTH(const int value)
    {
        assert(this->cap != nullptr);
        this->cap->set(cv::CAP_PROP_FRAME_WIDTH, value);
        this->_frame_width = value;
    }

    void StereoCamUsb::setCAP_PROP_FRAME_HEIGHT(const int value)
    {
        assert(this->cap != nullptr);
        this->cap->set(cv::CAP_PROP_FRAME_HEIGHT, value);
        this->_frame_height = value;
    }

    void StereoCamUsb::setCAP_PROP_AUTO_EXPOSURE(const int value)
    {
        assert(this->cap != nullptr);
        this->cap->set(cv::CAP_PROP_AUTO_EXPOSURE, value);
    }

    void StereoCamUsb::setCAP_PROP_EXPOSURE(const int value)
    {
        assert(this->cap != nullptr);
        this->cap->set(cv::CAP_PROP_EXPOSURE, value);
        this->_frame_exposure = value;
    }

    void StereoCamUsb::setCAP_PROP_FOURCC(char c1, char c2, char c3, char c4)
    {
        assert(this->cap != nullptr);
        auto fourcc = cv::VideoWriter::fourcc(c1, c2, c3, c4);
        this->cap->set(cv::CAP_PROP_FOURCC, fourcc);
    }

    void StereoCamUsb::setCAP_PROP_FPS(const int value)
    {
        assert(this->cap != nullptr);
        this->cap->set(cv::CAP_PROP_FPS, value);
        this->_frame_fps = value;
    }

    void StereoCamUsb::release()
    {
        this->cap->release();
    }

    int StereoCamUsb::get_frame_width()
    {
        return this->_frame_width;
    }

    int StereoCamUsb::get_frame_height()
    {
        return this->_frame_height;
    }

    int StereoCamUsb::get_frame_exposure()
    {
        return this->_frame_exposure;
    }

    int StereoCamUsb::get_frame_fps()
    {
        return this->_frame_fps;
    }

} // namespace stereo_cam_usb