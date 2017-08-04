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

/* Camera parameters */
CamConfig cfg;

/* Image Publisher */
image_transport::Publisher image_pub;

/* Camera resolution */
int height;
int width;

void imageCallback(ICameraFrame *frame) 
{
    /* Multiple by 1.5 because of YUV standard */
    cv::Mat img = cv::Mat(1.5 * height, width, CV_8UC1, frame->data);
    cv::cvtColor(img, img, CV_YUV420sp2RGB);

    // convert OpenCV image to ROS message
    cv_bridge::CvImage cvi;
    cvi.header.stamp = ros::Time::now();
    cvi.header.frame_id = "image";
    cvi.image = img;
    cvi.encoding = "rgb8";

    sensor_msgs::Image im;
    cvi.toImageMsg(im);
    image_pub.publish(im);

    img.release();
}

int main(int argc, char **argv)
{
    /* Ros init */
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("image_raw", 1);

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
        height = 2160;
        width = 3840;
    } else if (res == "1080p") {
        cfg.previewSize = CameraSizes::FHDSize();
        height = 1080;
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

    cfg.func = CAM_FUNC_HIRES;
        
    /* Program start */
    SnapCam cam(cfg);
    cam.setListener(imageCallback);

    /* Main loop */
    while(nh.ok()) {
        ros::spinOnce();
    }

    return 0;
}
