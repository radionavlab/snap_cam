#include "node.h"
#include <iostream>
#include <chrono>

Node::Node(int argc, char** argv) {
    ros::init(argc, argv, "~");
    this->nh_ = std::make_shared<ros::NodeHandle>("~");
    this->it_ = std::make_shared<image_transport::ImageTransport>(*(this->nh_));
    this->it_pub_ = this->it_->advertise("frame", 1);

    const CamConfig cfg = this->LoadCameraConfig();
    this->camera_ = std::make_shared<SnapCam>(cfg);
}

void Node::Start() {
    this->camera_->setListener(std::bind(&Node::FrameHandler, this, std::placeholders::_1));
    this->camera_->start();

    ros::Rate r(20);
    while(this->nh_->ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

void Node::FrameHandler(camera::ICameraFrame *frame) {
    // Check that an image is not already being processed
    if(this->is_busy_ == true) {
        std::cout << "busy" << std::endl;
        return;
    } else {
        this->is_busy_ = true;
    }

    { // Print FPS
        static long long last_time = 0;
        std::chrono::milliseconds now = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
        );

        std::cout << 1000.0/(now.count() - last_time) << " fps" << std::endl;
        last_time = now.count();
    }

    if(this->camera_->cameraType() == CAM_FORWARD) {
        // Convert YUV to RGB
        cv::Mat img = cv::Mat(1.5 * this->image_height_, this->image_width_, CV_8UC1, frame->data);
        cv::cvtColor(img, img, CV_YUV420sp2RGB);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        this->it_pub_.publish(msg);
    } else if(this->camera_->cameraType() == CAM_DOWN) {
        // cv::Mat img = cv::Mat(this->image_height_, this->image_width_, CV_8UC1, frame->data);
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
        // this->it_pub_.publish(msg);

        std::cout << this->image_width_ << ", " << this->image_height_ << std::endl;
        sensor_msgs::Image msg;
        msg.header       = std_msgs::Header();
        msg.height       = this->image_height_;
        msg.width        = this->image_width_;
        msg.encoding     = "mono8";
        msg.is_bigendian = true;
        msg.step         = this->image_width_;
        msg.data         = std::vector<uint8_t>(frame->data, frame->data + this->image_height_ * this->image_width_);
        this->it_pub_.publish(msg);

    }


    // Finished processing
    this->is_busy_ = false;
    
}

CamConfig Node::LoadCameraConfig() {
    CamConfig config;

    this->nh_->getParam("focus_mode",       config.focusMode);
    this->nh_->getParam("white_balance",    config.whiteBalance);
    this->nh_->getParam("iso",              config.ISO);
    this->nh_->getParam("preview_format",   config.previewFormat);
    this->nh_->getParam("brightness",       config.brightness);
    this->nh_->getParam("sharpness",        config.sharpness);
    this->nh_->getParam("contrast",         config.contrast);

    /* Set resolution size */
    /* Some of these are nonstandard! */
    std::string res;
    this->nh_->getParam("camera_resolution", res);
    if (res == "4k") {
        config.previewSize = CameraSizes::UHDSize();
        this->image_height_ = 2176;
        this->image_width_ = 3840;
    } else if (res == "1080p") {
        config.previewSize = CameraSizes::FHDSize();
        this->image_height_ = 1088;
        this->image_width_ = 1920;
    } else if (res == "720p") {
        config.previewSize = CameraSizes::HDSize();
        this->image_height_ = 736;
        this->image_width_ = 1280;
    } else if (res == "VGA") {
        config.previewSize = CameraSizes::VGASize();
        this->image_height_ = 480;
        this->image_width_ = 640;
    } else {
        ROS_ERROR("Invalid resolution %s!");
        exit(EXIT_FAILURE);
    }

    /* Camera type */
    std::string camera;
    this->nh_->getParam("camera", camera);
    if(camera == "forward") {
        config.cameraId        = 1;
        config.func            = CAM_FORWARD;
    } else if (camera == "down") {
        config.cameraId        = 0;
        config.func            = CAM_DOWN;
    } else {
        ROS_ERROR("Unknown camera!");
        exit(EXIT_FAILURE);
    }

    /* Exposure and gain cannot be set until camera is running */
    config.exposure        = 100;
    config.gain            = 50;

    return config;
}
