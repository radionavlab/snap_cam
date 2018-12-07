// Author: Tucker Haydon

#include "snap_cam.h"

#include <iostream>
#include <thread>

const CamConfig SnapCam::DEFAULT_CONFIG = {
  .cameraId       = 1,
  .focusMode      = "auto",
  .whiteBalance   = "auto",
  .ISO            = "ISO3200", 
  .previewFormat  = "yuv420sp",
  .sharpness      = 18,
  .brightness     = 3,
  .contrast       = 5,
  .exposure       = 100,
  .gain           = 50, 
  .previewSize    = CameraSizes::UHDSize(), 
  .func           = CAM_FORWARD,
};

SnapCam::SnapCam() : SnapCam(SnapCam::DEFAULT_CONFIG) {}

SnapCam::SnapCam(const CamConfig& cfg) {
    cb_=nullptr;
    this->Configure(cfg);
}

void SnapCam::Configure(const CamConfig& cfg) {
    config_ = cfg;
}

void SnapCam::Initialize() {

    std::cout << "Added Listner" << std::endl;
    // Ensure camera is connected and accessible
    if (camera::getNumberOfCameras() < 1) {
        std::cout << "No cameras detected. Are you using sudo?" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Create camera device
    if(camera::ICameraDevice::createInstance(config_.cameraId, &camera_) != 0) {
        std::cout << "Could not open camera." << std::endl;
        exit(EXIT_FAILURE);
    }

    // Add listener
    camera_->addListener(this);

    // Initialize camera device
    if(params_.init(camera_) != 0) {
        std::cout << "Failed to init parameters." << std::endl;
        camera::ICameraDevice::deleteInstance(&camera_);
        exit(EXIT_FAILURE);
    }

    if(config_.func == CAM_FORWARD) {
        // Set the image sizes
        params_.setPreviewSize(config_.previewSize); 
        params_.setVideoSize(config_.previewSize); 

        // Set image format. Pretty much only YUV420sp
        params_.setPreviewFormat(config_.previewFormat);

        // Leave these for 60 fps
        params_.setPreviewFpsRange(params_.getSupportedPreviewFpsRanges()[3]);
        params_.setVideoFPS(params_.getSupportedVideoFps()[0]);

        // Set picture parameters
        params_.setFocusMode(config_.focusMode);
        params_.setWhiteBalance(config_.whiteBalance);
        params_.setISO(config_.ISO);
        params_.setSharpness(config_.sharpness);
        params_.setBrightness(config_.brightness);
        params_.setContrast(config_.contrast);
    }

    if (params_.commit() != 0) {
        std::cout << "Commit failed" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void SnapCam::Start() {
  if(!this->running_) {
    this->Initialize();
    camera_->startPreview();
    camera_->startRecording();
    running_ = true;
  }
}

void SnapCam::Stop() {
  if(this->running_) {
    camera_->stopRecording();
    camera_->stopPreview();
    camera::ICameraDevice::deleteInstance(&camera_);
    running_ = false;
  }
}

SnapCam::~SnapCam() {
    this->Stop();
}

void SnapCam::onPreviewFrame(camera::ICameraFrame *frame) {
  if (!cb_ || config_.func != CAM_DOWN || this->busy_) { return; }
  this->busy_ = true;
  frame->acquireRef();
  cb_(frame);
  frame->releaseRef();
  this->busy_ = false;
}

void SnapCam::onVideoFrame(camera::ICameraFrame *frame) {
    if (!cb_ || config_.func != CAM_FORWARD || this->busy_) { return; }
    this->busy_ = true;
    frame->acquireRef();
    cb_(frame);
    frame->releaseRef();
    this->busy_ = false;
}

void SnapCam::SetListener(CallbackFunction fun) {
    cb_ = fun;
}
