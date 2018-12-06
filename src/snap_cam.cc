// Author: Tucker Haydon

#include "snap_cam.h"
#include "utils.h"

#include <iostream>
#include <thread>

const CamConfig SnapCam::DEFAULT_CONFIG = {
  .cameraId       = 1,
  .focusMode      = "continuous-video",
  .whiteBalance   = "auto",
  .ISO            = "ISO3200", 
  .previewFormat  = "yuv420sp",
  .sharpness      = 18,
  .brightness     = 3,
  .contrast       = 5,
  .exposure       = 100,
  .gain           = 50, 
  .previewSize    = CameraSizes::UHDSize(), 
  .pictureSize    = CameraSizes::UHDSize(),
  .func           = CAM_FORWARD,
};

SnapCam::SnapCam() : SnapCam(DEFAULT_CONFIG) {}

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
    // Start the camera preview and recording. Will start pushing frames asynchronously to the callbacks
    this->Initialize();
    camera_->startPreview();
    camera_->startRecording();
}

void SnapCam::Stop() {
    camera_->stopRecording();
    camera_->stopPreview();

    /* release camera device */
    camera::ICameraDevice::deleteInstance(&camera_);
}

SnapCam::~SnapCam() {
    this->Stop();
}

void SnapCam::onError() {
    std::cout << "Camera error!, aborting\n" << std::endl;
    exit(EXIT_FAILURE);
}

void SnapCam::onPreviewFrame(camera::ICameraFrame *frame) {
    if (!cb_ || config_.func != CAM_DOWN) { return; }
    cb_(frame);
}

void SnapCam::onVideoFrame(camera::ICameraFrame *frame) {
    if (!cb_ || config_.func != CAM_FORWARD) { return; }
    cb_(frame);
}

void SnapCam::SetListener(CallbackFunction fun) {
    cb_ = fun;
}
