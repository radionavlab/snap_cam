/* Copyright (c) 2016, PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of PX4 nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * SnapCam.cpp
 *
 *  Created on: Mar 16, 2016
 *      Author: Christoph, Nicolas
 *  Modified on: June 14, 2017
 *      Author: Tucker Haydon
 */

#include "SnapCam.h"

using namespace std;
using namespace camera;

SnapCam::SnapCam(CamConfig cfg)
{
    cb_=nullptr;
    initialize(cfg);
}

int SnapCam::initialize(CamConfig cfg)
{
    int PROBLEM_EXIT = -1;

    // Ensure camera is connected and accessible
    if (getNumberOfCameras() < 1) {
        printf("No cameras detected. Are you using sudo?\n");
        return PROBLEM_EXIT;
    }
    cfg.cameraId = 1;

    // Create camera device
    if(ICameraDevice::createInstance(cfg.cameraId, &camera_) != 0) {
        printf("Could not open camera.");
        return PROBLEM_EXIT;
    }

    // Add listener
    camera_->addListener(this);

    // Initialize camera device
    if(params_.init(camera_) != 0) {
        printf("failed to init parameters\n");
        ICameraDevice::deleteInstance(&camera_);
        return PROBLEM_EXIT;
    }

    // Set the image sizes
    params_.setPreviewSize(cfg.previewSize); 
    params_.setVideoSize(cfg.previewSize); 

    // Set image format. Pretty much only YUV420sp
    params_.setPreviewFormat(cfg.previewFormat);

    // Leave these for 60 fps
    params_.setPreviewFpsRange(params_.getSupportedPreviewFpsRanges()[3]);
    params_.setVideoFPS(params_.getSupportedVideoFps()[0]);

    // Set picture parameters
    params_.setFocusMode(cfg.focusMode);
    params_.setWhiteBalance(cfg.whiteBalance);
    params_.setISO(cfg.ISO);
    params_.setSharpness(cfg.sharpness);
    params_.setBrightness(cfg.brightness);
    params_.setContrast(cfg.contrast);

    if (params_.commit() != 0) {
        printf("Commit failed\n");
        return PROBLEM_EXIT;
    }

    // Start the camera preview and recording. Will start pushing frames asynchronously to the callbacks
    camera_->startPreview();
    camera_->startRecording();

    config_ = cfg;
}

SnapCam::~SnapCam() 
{
    camera_->stopRecording();
    camera_->stopPreview();

    /* release camera device */
    ICameraDevice::deleteInstance(&camera_);
}

void SnapCam::onError() 
{
    printf("camera error!, aborting\n");
    exit(EXIT_FAILURE);
}

void SnapCam::onPreviewFrame(ICameraFrame *frame) {}

void SnapCam::onVideoFrame(ICameraFrame *frame) 
{
    static uint32_t count = 0;

    if (!cb_) { return; }
    frame->acquireRef();
    std::thread(cb_, frame).detach();
    
    static long long lastTime = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long currentTime = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000L;
    cout << "Video FPS: " << (1000.0 / (double)(currentTime - lastTime)) << endl;
    lastTime = currentTime;
}

void SnapCam::setListener(CallbackFunction fun)
{
    cb_ = fun;
}
