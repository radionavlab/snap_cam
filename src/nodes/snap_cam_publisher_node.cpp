/*
*
*  Created on: Mar 16, 2016
*      Author: Nicolas
*/

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <errno.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <typeinfo>
#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>

#include "SnapCam.h"

ros::Publisher image_pub;

void imageCallback(unsigned char *buffer, int size) {
        static int count = 0;
 
        std::vector<uint8_t> data(buffer, buffer + size);
        sensor_msgs::CompressedImage msg;

        msg.header.stamp=ros::Time::now();
        msg.header.frame_id="image";
        msg.format="jpeg";
        msg.data = data;

	image_pub.publish(msg);

        count++;
}

int main(int argc, char **argv)
{

        /* Ros init */
	ros::init(argc, argv, "snap_cam_highres/publisher");
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);
        image_pub = nh.advertise<sensor_msgs::CompressedImage>("image_raw/compressed", 1);


        /* Camera parameters */
	CamConfig cfg;
	cfg.func = CAM_FUNC_HIRES;

        /* auto, infinity, macro, continuous-video, continuous-picture, manual */
	if (!nh.getParam("focus_mode", cfg.focusMode)) {
		cfg.focusMode="auto";
		ROS_WARN("Defaulting to auto focus mode.");
	}

        /* auto, incandescent, fluorescent, warm-fluorescent, daylight, cloudy-daylight, twilight, shade, manual-cct */
	if (!nh.getParam("white_balance", cfg.whiteBalance)) {
		cfg.whiteBalance="auto";
		ROS_WARN("Defaulting to auto white balance.");
	}

        /* auto, ISO_HJR, ISO100, ISO200, ISO400, ISO800, ISO1600, ISO3200 */
	if (!nh.getParam("iso", cfg.ISO)) {
		cfg.ISO="auto";
		ROS_WARN("Defaulting to auto ISO.");
	}

        /* yuv420sp, yuv420p, nv12-venus, bayer-rggb */
	if (!nh.getParam("preview_format", cfg.previewFormat)) {
		cfg.previewFormat="yuv420sp";
		ROS_WARN("Defaulting to yuv420sp preview format.");
	}

        /* 1-6 in increments of 1 */
	if (!nh.getParam("brightness", cfg.brightness)) {
		cfg.brightness=3;
		ROS_WARN("Defaulting to 3 brightness");
	}

        /* 0-36 in increments of 6 */
	if (!nh.getParam("sharpness", cfg.sharpness)) {
		cfg.sharpness=18;
		ROS_WARN("Defaulting to 18 sharpness");
	}

        /* 1-10 in increments of 1 */
	if (!nh.getParam("contrast", cfg.contrast)) {
		cfg.contrast=5;
		ROS_WARN("Defaulting to 5 contrast.");
	}

        /* QVGA, VGA, 720p, 1080p, 4k */
	std::string res;
	if (!nh.getParam("camera_resolution", res)) {
		res = "VGA";
		ROS_WARN("No resolution parameter provided. Defaulting to %s.", res.c_str());
	}

        /* Set resolution size */
	if (res == "4k") {
		cfg.previewSize = CameraSizes::UHDSize();
		cfg.pictureSize = CameraSizes::UHDSize();
	} else if (res == "1080p") {
		cfg.previewSize = CameraSizes::FHDSize();
		cfg.pictureSize = CameraSizes::FHDSize();
	} else if (res == "720p") {
		cfg.previewSize = CameraSizes::HDSize();
		cfg.pictureSize = CameraSizes::HDSize();
	} else if (res == "VGA") {
		cfg.previewSize = CameraSizes::VGASize();
		cfg.pictureSize = CameraSizes::VGASize();
	} else if (res == "QVGA") {
		cfg.previewSize = CameraSizes::QVGASize();
		cfg.pictureSize = CameraSizes::QVGASize();
	} else if (res == "stereoVGA") {
		cfg.previewSize = CameraSizes::stereoVGASize();
		cfg.pictureSize = CameraSizes::stereoVGASize();
	} else if (res == "stereoQVGA") {
		cfg.previewSize = CameraSizes::stereoQVGASize();
		cfg.pictureSize = CameraSizes::stereoQVGASize();
	} else {
		ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
		cfg.previewSize = CameraSizes::stereoVGASize();
		cfg.pictureSize = CameraSizes::stereoVGASize();
	}

        
        /* Program start */
	SnapCam cam(cfg);
	cam.setListener(imageCallback);

        /* Main loop */
        while(nh.ok()) {
            cam.takePicture();
	    ros::spinOnce();
        }

	return 0;
}
