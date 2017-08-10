/*
*
*  Created on: Mar 16, 2016
*      Author: Nicolas
*      Modified: Tucker Haydon
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

#include "SnapCam.h"

/* Camera parameters */
CamConfig cfg;

/* Image Publisher */
ros::Publisher image_pub;

/* Camera resolution */
int height;
int width;

void imageCallback(ICameraFrame *frame) 
{
    static int seq = 0;

    // Convert YUV to RGB
    /* Multiple by 1.5 because of YUV standard */
    cv::Mat img = cv::Mat(1.5 * height, width, CV_8UC1, frame->data);
    cv::cvtColor(img, img, CV_YUV420sp2RGB);

    // Compress to JPEG
    std::vector<uint8_t> buff;//buffer for coding
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 100;
    cv::imencode(".jpg", img, buff, param);
    img.release();

    // Compress ros message
    sensor_msgs::CompressedImage im;
    im.format="jpeg";
    im.data = buff;
    im.header.seq = seq++;
    im.header.stamp=ros::Time::now();
    im.header.frame_id="";

    // Publish
    image_pub.publish(im);
}

int main(int argc, char **argv)
{
    /* Ros init */
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh("~");
    image_pub = nh.advertise<sensor_msgs::CompressedImage>("image/compressed", 1);

    if (!nh.getParam("focus_mode", cfg.focusMode)) {
        cfg.focusMode="auto";
        ROS_WARN("Defaulting to auto focus mode.");
    }

    if (!nh.getParam("white_balance", cfg.whiteBalance)) {
        cfg.whiteBalance="auto";
        ROS_WARN("Defaulting to auto white balance.");
    }

    if (!nh.getParam("iso", cfg.ISO)) {
        cfg.ISO="auto";
        ROS_WARN("Defaulting to auto ISO.");
    }

    if (!nh.getParam("preview_format", cfg.previewFormat)) {
        cfg.previewFormat="yuv420sp";
        ROS_WARN("Defaulting to yuv420sp preview format.");
    }

    if (!nh.getParam("brightness", cfg.brightness)) {
        cfg.brightness=3;
        ROS_WARN("Defaulting to 3 brightness");
    }

    if (!nh.getParam("sharpness", cfg.sharpness)) {
        cfg.sharpness=18;
        ROS_WARN("Defaulting to 18 sharpness");
    }

    if (!nh.getParam("contrast", cfg.contrast)) {
        cfg.contrast=5;
        ROS_WARN("Defaulting to 5 contrast.");
    }

    std::string res;
    if (!nh.getParam("camera_resolution", res)) {
        res = "VGA";
        ROS_WARN("No resolution parameter provided. Defaulting to %s.", res.c_str());
    }

    /* Set resolution size */
    // Some of these are nonstandard!
    if (res == "4k") {
        cfg.previewSize = CameraSizes::UHDSize();
        height = 2160;
        width = 3840;
    } else if (res == "1080p") {
        cfg.previewSize = CameraSizes::FHDSize();
        height = 1088;
        width = 1920;
    } else if (res == "720p") {
        cfg.previewSize = CameraSizes::HDSize();
        height = 720;
        width = 1280;
    } else if (res == "VGA") {
        cfg.previewSize = CameraSizes::VGASize();
        height = 480;
        width = 640;
    } else {
        ROS_ERROR("Invalid resolution %s. Defaulting to VGA\n", res.c_str());
        cfg.previewSize = CameraSizes::stereoVGASize();
    }

    // Only using hires camera
    cfg.func = CAM_FUNC_HIRES;
        
    /* Program start */
    SnapCam cam(cfg);
    cam.setListener(imageCallback);

    /* Main loop */
    ros::Rate r(10);
    while(nh.ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
