#include "node.h"
#include "utils.h"
#include <iostream>


Node::Node(int argc, char** argv) {
    ros::init(argc, argv, "~");
    this->nh_ = std::make_shared<ros::NodeHandle>("~");

    const CamConfig cfg = this->LoadCameraConfig();
    this->camera_ = std::make_shared<SnapCam>(cfg);
    this->camera_server_ = std::make_shared<snapcam::CameraServer>("/tmp/camera_server");
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
        return;
    } else {
        this->is_busy_ = true;
    }

    // { // Print FPS
    //     static long long last_time = 0;
    //     long long now = CurrentTimeMillis();
    //     if(now - last_time < 1000.0/15.0) { this->is_busy_ = false; return; }
    //     else {
    //         std::cout << 1000.0/(now - last_time) << " fps" << std::endl;
    //         last_time = now;
    //     }
    // }

    if(this->camera_->cameraType() == CAM_FORWARD) {
            this->camera_server_->PublishFrame(frame->fd, frame->size, this->image_width_, this->image_height_);
    } else if(this->camera_->cameraType() == CAM_DOWN) {
        std::cout << "This feature not yet implemented! Images can stream, but the bytes are out of order. Please fix this in node.cc!" << std::endl;
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
