/*
* optical_flow.cpp
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
	ros::init(argc, argv, "snap_cam_highres/publisher");
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);
        image_pub = nh.advertise<sensor_msgs::CompressedImage>("image_raw/compressed", 1);

	std::string res;

	if (!nh.getParam("camera_resolution", res)) {
		res = "VGA";
		ROS_WARN("No resolution parameter provided. Defaulting to %s.", res.c_str());
	}

	std::string camera_type;

	if (!nh.getParam("camera_type", camera_type)) {
		camera_type = "highres";
		ROS_WARN("No camera type parameter provided. Defaulting to %s.", camera_type.c_str());
	}

	int camera_fps_idx;

	if (!nh.getParam("camera_fps_idx", camera_fps_idx)) {
		camera_fps_idx = 0;
		ROS_WARN("No camera fps idx parameter provided. Defaulting to %d.", camera_fps_idx);
	}

	CamConfig cfg;

	if (camera_type == "highres") {
		cfg.func = CAM_FUNC_HIRES;

	} else if (camera_type == "optflow") {
		cfg.func = CAM_FUNC_OPTIC_FLOW;

	} else {
		ROS_ERROR("Invalid camera type %s. Defaulting to highres.", camera_type.c_str());
		cfg.func = CAM_FUNC_HIRES;
	}

	if (res == "4k") {
		cfg.pSize = CameraSizes::UHDSize();

	} else if (res == "1080p") {
		cfg.pSize = CameraSizes::FHDSize();

	} else if (res == "720p") {
		cfg.pSize = CameraSizes::HDSize();

	} else if (res == "VGA") {
		cfg.pSize = CameraSizes::VGASize();

	} else if (res == "QVGA") {
		cfg.pSize = CameraSizes::QVGASize();

	} else if (res == "stereoVGA") {
		cfg.pSize = CameraSizes::stereoVGASize();

	} else if (res == "stereoQVGA") {
		cfg.pSize = CameraSizes::stereoQVGASize();

	} else {
		ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
		cfg.pSize = CameraSizes::stereoVGASize();
	}

	cfg.fps = camera_fps_idx;

	SnapCam cam(cfg);
	cam.setListener(imageCallback);

        while(nh.ok()) {
            cam.takePicture();
	    ros::spinOnce();
        }

	return 0;
}
