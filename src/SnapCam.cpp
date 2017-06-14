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
 */

#include "SnapCam.h"

#include <sstream>
#include <vector>

using namespace std;
using namespace camera;

static uint64_t get_absolute_time();

SnapCam::SnapCam(CamConfig cfg)
	: cb_(nullptr),
	auto_exposure_(false)
{
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

	int pFpsIdx;
	int vFpsIdx;

	pSize_ = cfg.pSize;

	rc = setFPSindex(cfg.fps, pFpsIdx, vFpsIdx);

	if ( rc == -1)
	{
		printf("FPS indexing failed pFpsIdx: %d  vFpsIdx: %d\n", pFpsIdx, vFpsIdx);
		return rc;
	}

        /* auto, infinity, macro, continuous-video, continuous-picture, manual */
	params_.setFocusMode("continuous-picture");

        /* auto, incandescent, fluorescent, warm-fluorescent, daylight, cloudy-daylight, twilight, shade, manual-cct */
	params_.setWhiteBalance("fluorescent");

        /* auto, ISO_HJR, ISO100, ISO200, ISO400, ISO800, ISO1600, ISO3200 */
	params_.setISO("ISO3200");

        /* yuv420sp, yuv420p, nv12-venus, bayer-rggb */
	params_.setPreviewFormat("yuv420sp");


	params_.setPreviewSize(pSize_);
	params_.setPictureSize(pSize_);
	params_.setPreviewFpsRange(caps_.previewFpsRanges[pFpsIdx]);

	rc = params_.commit();
	if (rc) {
		printf("Commit failed\n");
		return rc;
	}

	camera_->startPreview();
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
	pthread_mutex_lock(&mutexPicDone);
	while(isPicDone == false){
    		pthread_cond_wait(&cvPicDone, &mutexPicDone);
	}
	isPicDone=false;
	pthread_mutex_unlock(&mutexPicDone);

	usleep(100000);
        this->camera_->takePicture();
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
void SnapCam::setListener(CallbackFunction fun, T *obj)
{
	cb_ = std::bind(fun, obj);
}

void SnapCam::updateExposure(cv::Mat &frame)
{
	//limit update rate to 5Hz
	static int counter = 1;
	static int devider = std::round(config_.fps*0.2);
	if (counter%devider != 0) {
		counter++;
		return;
	}
	counter = 1;

	//init histogram variables
	static cv::Mat hist;
	static int channels[] = {0};
	static int histSize[] = {10}; //10 bins
	static float range[] = { 0, 255 };
	static const float* ranges[] = { range };

	// only use 128x128 window to calculate exposure
	static int mask_size = 128;
	static cv::Mat mask(frame.rows,frame.cols,CV_8U,cv::Scalar(0));
	static bool mask_set = false;
	if (!mask_set) {
		mask(cv::Rect(frame.cols/2-mask_size/2,frame.rows/2-mask_size/2,mask_size,mask_size)) = 255;
		mask_set = true;
	}

	//calculate the histogram with 10 bins
	calcHist( &frame, 1, channels, mask,
	     hist, 1, histSize, ranges,
	     true, // the histogram is uniform
	     false );

	//calculate Mean Sample Value (MSV)
	float msv = 0.0f;
	for (int i = 0; i < histSize[0]; i++) {
		msv += (i+1)*hist.at<float>(i)/16384.0f; //128x128 -> 16384
	}

	//get first exposure value
	static float exposure = config_.exposureValue;
	static float exposure_old = config_.exposureValue;

	//MSV target value
	static const float msv_target = 5.0f;

	//PID-controller
	static const float P_gain = 30.0f;
	static const float I_gain = 0.1f;
	static const float D_gain = 0.1f;

	float msv_error = msv_target - msv;
	static float msv_error_old = msv_error;
	static float msv_error_int = 0.0f;
	msv_error_int += msv_error;
	float msv_error_d = msv_error - msv_error_old;

	exposure += P_gain*msv_error + I_gain*msv_error_int + D_gain*msv_error_d;

	if (exposure < 1.0f)
		exposure = 1.0f;
	if (exposure > 511.0f)
		exposure = 511.0f;

	msv_error_old = msv_error;

	//set new exposure value if bigger than threshold
	if (fabs(exposure - exposure_old) > EXPOSURE_CHANGE_THRESHOLD) {
		params_.setManualExposure(std::round(exposure));
		params_.commit();
		exposure_old = exposure;
	}

}

/**
 *  FUNCTION: setDefaultConfig
 *
 *  set default config based on camera module
 *
 * */
static int setDefaultConfig(CamConfig &cfg)
{

	cfg.outputFormat = YUV_FORMAT;
	cfg.dumpFrames = false;
	cfg.runTime = 10;
	cfg.infoMode = false;
	cfg.testVideo = true;
	cfg.testSnapshot = false;
	cfg.exposureValue = DEFAULT_EXPOSURE_VALUE;  /* Default exposure value */
	cfg.gainValue = DEFAULT_GAIN_VALUE;  /* Default gain value */
	cfg.fps = DEFAULT_CAMERA_FPS;
	cfg.picSizeIdx = -1;
	cfg.logLevel = CAM_LOG_SILENT;
	cfg.snapshotFormat = JPEG_FORMAT;

	switch (cfg.func) {
	case CAM_FUNC_OPTIC_FLOW:
		cfg.pSize   = CameraSizes::VGASize();
		cfg.vSize   = CameraSizes::VGASize();
		cfg.picSize   = CameraSizes::VGASize();
		cfg.outputFormat = RAW_FORMAT;
		break;

	case CAM_FUNC_RIGHT_SENSOR:
		cfg.pSize   = CameraSizes::VGASize();
		cfg.vSize   = CameraSizes::VGASize();
		cfg.picSize   = CameraSizes::VGASize();
		break;

	case CAM_FUNC_STEREO:
		cfg.pSize = CameraSizes::stereoVGASize();
		cfg.vSize  = CameraSizes::stereoVGASize();
		cfg.picSize  = CameraSizes::stereoVGASize();
		break;

	case CAM_FUNC_HIRES:
		cfg.pSize = CameraSizes::FHDSize();
		cfg.vSize = CameraSizes::HDSize();
		cfg.picSize = CameraSizes::FHDSize();
		break;

	default:
		printf("invalid sensor function \n");
		break;
	}

}

static uint64_t get_absolute_time()
{
	struct timespec time;

	uint64_t micros = 0;

	clock_gettime(CLOCK_MONOTONIC, &time);
	micros = (uint64_t)time.tv_sec * 1000000 + time.tv_nsec / 1000;
	return micros;
}
