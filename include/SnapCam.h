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

#pragma once
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <mutex>
#include <vector>
#include <sstream>
#include <thread>

#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <typeinfo>
#include <iostream>

#include "camera.h"
#include "camera_log.h"
#include "camera_parameters.h"

using namespace std;
using namespace camera;

// workaround to only have to define these once
struct CameraSizes {
	static ImageSize FourKSize()        {static ImageSize is(4096, 2160); return is;};
	static ImageSize UHDSize()          {static ImageSize is(3840, 2160); return is;};
	static ImageSize FHDSize()          {static ImageSize is(1920, 1080); return is;};
	static ImageSize HDSize()           {static ImageSize is(1280, 720); return is;};
	static ImageSize VGASize()          {static ImageSize is(640, 480); return is;};
	static ImageSize stereoVGASize()    {static ImageSize is(1280, 480); return is;};
	static ImageSize QVGASize()         {static ImageSize is(320, 240); return is;};
	static ImageSize stereoQVGASize()   {static ImageSize is(640, 240); return is;};
};

enum CamFunction {
    CAM_FUNC_UNKNOWN = -1,
    CAM_FUNC_HIRES = 0,
    CAM_FUNC_OPTIC_FLOW = 1,
    CAM_FUNC_RIGHT_SENSOR = 2,
    CAM_FUNC_STEREO = 3,
};

/**
*  Helper class to store all parameter settings
*/
struct CamConfig {
        int cameraId;
        string focusMode;
        string whiteBalance;
        string ISO;
        string previewFormat;
        int sharpness;
        int brightness;
        int contrast;
	int exposure;
	int gain;
	ImageSize previewSize;
	ImageSize pictureSize;
        int func;
};

// Callback function.
typedef std::function<void(ICameraFrame *frame)> CallbackFunction;

/**
 * CLASS  SnapCam
 *
 * inherits ICameraListers which provides core functionality
 */
class SnapCam : ICameraListener
{
public:

	SnapCam(CamConfig cfg);
	~SnapCam();

	void setListener(CallbackFunction fun);  // register a function callback
        void start();

	/* listener methods */
	virtual void onError();
        virtual void onVideoFrame(ICameraFrame *frame);
        virtual void onPreviewFrame(ICameraFrame *frame);

private:
	int initialize(CamConfig cfg);

	ICameraDevice *camera_;
	CameraParams params_;
	CamConfig config_;
	CallbackFunction cb_;
};

