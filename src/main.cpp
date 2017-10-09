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
#include <sys/stat.h>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

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

/* Save Directory for images */
std::string save_directory;

void writer(ICameraFrame *frame) 
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

    // Write
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << seq;
    std::string filename = "frame" + ss.str() + ".jpg";
    std::string fullPath = save_directory + filename;
    std::ofstream savefile(fullPath.c_str(), ios::out | ios::binary);
    savefile.write((const char*)&buff[0], buff.size());
    savefile.close();

    std::string command = "echo '" + filename + " 1 2 3 4 5 6' >> " + save_directory + "/image_poses.txt";
    std::system(command.c_str());
    seq++;
}

void publisher(ICameraFrame *frame) 
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
    im.header.seq = seq;
    im.header.stamp=ros::Time::now();
    im.header.frame_id="";

    // Publish
    image_pub.publish(im);

    seq++;
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

    std::string base_directory;
    if (!nh.getParam("base_directory", base_directory)) {
        base_directory = "/home/linaro/";
        ROS_WARN("No save directory provided. Defaulting to home directory.");
    }

    /* Make a directory for all of the images */
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string year = std::to_string(1900 + ltm->tm_year);
    std::string month = std::to_string(1 + ltm->tm_mon);
    std::string day = std::to_string(ltm->tm_mday);
    std::string hour = std::to_string(ltm->tm_hour);
    std::string min = std::to_string(ltm->tm_min);
    std::string sec = std::to_string(ltm->tm_sec);
    std::string dir = year + "-" + month + "-" + day + "-" + hour + "-" + min + "-" + sec + "/";
    save_directory = base_directory + dir;
    std::cout << save_directory << std::endl;
    const int dir_err = mkdir(save_directory.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == dir_err){
        ROS_WARN("Error creating directory!");
        exit(1);
    }

    std::string callback_mode;
    if (!nh.getParam("callback_mode", callback_mode)) {
        callback_mode = "publish";
        ROS_WARN("No callback_mode provided. Defaulting to publishing.");
    }

    /* Set resolution size */
    // Some of these are nonstandard!
    if (res == "4k") {
        cfg.previewSize = CameraSizes::UHDSize();
        height = 2176;
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

    /* Set the callback mode */
    if(callback_mode == "write") {
        cam.setListener(writer);
    } else {
        cam.setListener(publisher);
    }

    /* Main loop */
    ros::Rate r(10);
    while(nh.ok()) {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
