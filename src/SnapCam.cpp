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

static uint64_t get_absolute_time();

SnapCam::SnapCam(CamConfig cfg)
{
        cb_=nullptr;
	isPicDone=true;
	pthread_mutex_init(&mutexPicDone, 0);
    	pthread_cond_init(&cvPicDone, 0);

	initialize(cfg);
}

int SnapCam::findCamera(CamConfig cfg, int32_t &camera_id)
{
	int num_cams = camera::getNumberOfCameras();

	if (num_cams < 1) {
		printf("No cameras detected. Exiting.\n");
		return -1;
	}

	bool found = false;

	for (int i = 0; i < num_cams; ++i) {
		camera::CameraInfo info;
		getCameraInfo(i, info);
		if (info.func == static_cast<int>(cfg.func)) {
			camera_id = i;
			found = true;
		}
	}

	if (!found) {
		printf("Could not find camera of type %d. Exiting", cfg.func);
		return -1;
	}

	printf("Camera of type %d has ID = %d\n", cfg.func, camera_id);

	return 0;
}

int SnapCam::initialize(CamConfig cfg)
{
	int rc;
	int32_t cameraId;
	rc = SnapCam::findCamera(cfg, cameraId);

	if (rc != 0) {
		printf("Cannot find camera Id for type: %d", cfg.func);
		return rc;
	}
	cfg.cameraId = cameraId;

	rc = ICameraDevice::createInstance(cfg.cameraId, &camera_);

	if (rc != 0) {
		printf("Could not open camera %d\n", cfg.func);
		return rc;
	}

	camera_->addListener(this);

	rc = params_.init(camera_);

	if (rc != 0) {
		printf("failed to init parameters\n");
		ICameraDevice::deleteInstance(&camera_);
		return rc;
	}

	/* query capabilities */
	caps_.pSizes = params_.getSupportedPreviewSizes();
	caps_.vSizes = params_.getSupportedVideoSizes();
	caps_.picSizes = params_.getSupportedPictureSizes();
	caps_.focusModes = params_.getSupportedFocusModes();
	caps_.wbModes = params_.getSupportedWhiteBalance();
	caps_.isoModes = params_.getSupportedISO();
	caps_.brightness = params_.getSupportedBrightness();
	caps_.sharpness = params_.getSupportedSharpness();
	caps_.contrast = params_.getSupportedContrast();
	caps_.previewFpsRanges = params_.getSupportedPreviewFpsRanges();
	caps_.videoFpsValues = params_.getSupportedVideoFps();
	caps_.previewFormats = params_.getSupportedPreviewFormats();
	caps_.rawSize = params_.get("raw-size");
	printCapabilities();
        
	params_.setPreviewSize(cfg.previewSize);
	params_.setPictureSize(cfg.pictureSize);
	params_.setFocusMode(cfg.focusMode);
	params_.setWhiteBalance(cfg.whiteBalance);
	params_.setISO(cfg.ISO);
        params_.setSharpness(cfg.sharpness);
        params_.setBrightness(cfg.brightness);
        params_.setContrast(cfg.contrast);
	params_.setPreviewFormat(cfg.previewFormat);

        // params_.set("preview-format", "bayer-rggb");
        // params_.set("picture-format", "bayer-mipi-10bggr");
        // params_.set("picture-format", "bayer-mipi-10gbrg");
        // params_.set("raw-size", "1920x1080");

	rc = params_.commit();
	if (rc) {
		printf("Commit failed\n");
		return rc;
	}

        /* Must start preview before setting manual gain and exposure */
	camera_->startPreview();

        params_.setManualGain(cfg.gain);
        params_.setManualExposure(cfg.exposure);
        // params_.setVerticalFlip(true);
        // params_.setHorizontalMirror(false);

	rc = params_.commit();
	if (rc) {
		printf("Commit failed\n");
		return rc;
	}

	config_ = cfg;

}

SnapCam::~SnapCam()
{
	camera_->stopPreview();

	/* release camera device */
	ICameraDevice::deleteInstance(&camera_);
}

void SnapCam::onError()
{
	printf("camera error!, aborting\n");
	exit(EXIT_FAILURE);
}

void SnapCam::onPictureFrame(ICameraFrame *frame) {

	// No need to send the image anywhere
	if (!cb_) { return; }

	cb_(frame->data, frame->size);

	pthread_mutex_lock(&mutexPicDone);
	isPicDone=true;
	pthread_cond_signal(&cvPicDone);
	pthread_mutex_unlock(&mutexPicDone);
}

void SnapCam::takePicture() {

        usleep(config_.sleepTime);
        pthread_mutex_lock(&mutexPicDone);
        isPicDone = false;
        camera_->takePicture();

	while(isPicDone == false){
    		pthread_cond_wait(&cvPicDone, &mutexPicDone);
	}
	pthread_mutex_unlock(&mutexPicDone);
/*
        struct timeval tp;
        gettimeofday(&tp, NULL);
        long long currentTime = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000;
        static long long lastTime = currentTime;

        long long timeToSleepMs = 0;
        long long leftover = 900 - (currentTime - lastTime);
        if(leftover > 0) {
            timeToSleepMs = leftover;
        }

	usleep(timeToSleepMs * 1000);

*/
        // usleep(100000);

        // lastTime = currentTime;
}


int SnapCam::printCapabilities()
{
	printf("Camera capabilities\n");

	printf("available preview sizes:\n");

	for (int i = 0; i < caps_.pSizes.size(); i++) {
		printf("%d: %d x %d\n", i, caps_.pSizes[i].width, caps_.pSizes[i].height);
	}

	printf("available video sizes:\n");

	for (int i = 0; i < caps_.vSizes.size(); i++) {
		printf("%d: %d x %d\n", i, caps_.vSizes[i].width, caps_.vSizes[i].height);
	}

	printf("available picture sizes:\n");

	for (int i = 0; i < caps_.picSizes.size(); i++) {
		printf("%d: %d x %d\n", i, caps_.picSizes[i].width, caps_.picSizes[i].height);
	}

	printf("available preview formats:\n");

	for (int i = 0; i < caps_.previewFormats.size(); i++) {
		printf("%d: %s\n", i, caps_.previewFormats[i].c_str());
	}

	printf("available focus modes:\n");

	for (int i = 0; i < caps_.focusModes.size(); i++) {
		printf("%d: %s\n", i, caps_.focusModes[i].c_str());
	}

	printf("available whitebalance modes:\n");

	for (int i = 0; i < caps_.wbModes.size(); i++) {
		printf("%d: %s\n", i, caps_.wbModes[i].c_str());
	}

	printf("available ISO modes:\n");

	for (int i = 0; i < caps_.isoModes.size(); i++) {
		printf("%d: %s\n", i, caps_.isoModes[i].c_str());
	}

	printf("available brightness values:\n");
	printf("min=%d, max=%d, step=%d\n", caps_.brightness.min,
	       caps_.brightness.max, caps_.brightness.step);
	printf("available sharpness values:\n");
	printf("min=%d, max=%d, step=%d\n", caps_.sharpness.min,
	       caps_.sharpness.max, caps_.sharpness.step);
	printf("available contrast values:\n");
	printf("min=%d, max=%d, step=%d\n", caps_.contrast.min,
	       caps_.contrast.max, caps_.contrast.step);

	printf("available preview fps ranges:\n");

	for (int i = 0; i < caps_.previewFpsRanges.size(); i++) {
		printf("%d: [%d, %d]\n", i, caps_.previewFpsRanges[i].min,
		       caps_.previewFpsRanges[i].max);
	}

	printf("available video fps values:\n");

	for (int i = 0; i < caps_.videoFpsValues.size(); i++) {
		printf("%d: %d\n", i, caps_.videoFpsValues[i]);
	}

	return 0;
}

/**
 * FUNCTION: setFPSindex
 *
 * scans through the supported fps values and returns index of
 * requested fps in the array of supported fps
 *
 * @param fps      : Required FPS  (Input)
 * @param pFpsIdx  : preview fps index (output)
 * @param vFpsIdx  : video fps index   (output)
 *
 *  */
int SnapCam::setFPSindex(int fps, int &pFpsIdx, int &vFpsIdx)
{
	int defaultPrevFPSIndex = -1;
	int defaultVideoFPSIndex = -1;
	int i, rc = 0;

	for (i = 0; i < caps_.previewFpsRanges.size(); i++) {
		if ((caps_.previewFpsRanges[i].max) / 1000 == fps) {
			pFpsIdx = i;
			break;
		}

		if ((caps_.previewFpsRanges[i].max) / 1000 == DEFAULT_CAMERA_FPS) {
			defaultPrevFPSIndex = i;
		}
	}

	if (i >= caps_.previewFpsRanges.size()) {
		if (defaultPrevFPSIndex != -1) {
			pFpsIdx = defaultPrevFPSIndex;

		} else {
			pFpsIdx = -1;
			rc = -1;
		}
	}

	for (i = 0; i < caps_.videoFpsValues.size(); i++) {
		if (fps == 30 * caps_.videoFpsValues[i]) {
			vFpsIdx = i;
			break;
		}

		if (DEFAULT_CAMERA_FPS == 30 * caps_.videoFpsValues[i]) {
			defaultVideoFPSIndex = i;
		}
	}

	if (i >= caps_.videoFpsValues.size()) {
		if (defaultVideoFPSIndex != -1) {
			vFpsIdx = defaultVideoFPSIndex;

		} else {
			vFpsIdx = -1;
			rc = -1;
		}
	}

	return rc;
}

/**
 *  FUNCTION: setListener
 *
 *  Set a listener for image callbacks
 *  The callback is provided with a cv::Mat image
 *
 * */
void SnapCam::setListener(CallbackFunction fun)
{
	cb_ = fun;
}

/**
 *  FUNCTION: setListener
 *
 *  Set a class member listener for image callbacks
 *  The callback is provided with a cv::Mat image
 *
 * */
template <class T>
void SnapCam::setListener(CallbackFunction fun, T *obj) {
	cb_ = std::bind(fun, obj);
}

static uint64_t get_absolute_time() {
	struct timespec time;

	uint64_t micros = 0;

	clock_gettime(CLOCK_MONOTONIC, &time);
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec / 1000;
	return micros;
}
