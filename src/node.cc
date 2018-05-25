#include "node.h"
#include "utils.h"
#include "socket_io.h"
#include <iostream>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <thread>

#include <sys/mman.h>
#include <sys/stat.h>        /* For mode constants */
#include <fcntl.h>  

Node::Node(int argc, char** argv) {
    ros::init(argc, argv, "~");
    this->nh_ = std::make_shared<ros::NodeHandle>("~");
    this->it_ = std::make_shared<image_transport::ImageTransport>(*(this->nh_));
    this->it_pub_ = this->it_->advertise("frame", 1);

    const CamConfig cfg = this->LoadCameraConfig();
    this->camera_ = std::make_shared<SnapCam>(cfg);
}

void Node::Start() {
    this->camera_->setListener(std::bind(&Node::FrameHandler, this, std::placeholders::_1));
    this->camera_->start();

    ros::Rate r(20);
    while(this->nh_->ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

void Node::FrameHandler(camera::ICameraFrame *frame) {
    {
        static int counter = 0;
        if(counter++ > 0) return;
    }

    {
        std::cout << getpid() << std::endl;
        std::cout << frame->fd << std::endl;
        std::cout << frame->size << std::endl;
        for(size_t i = 0; i < 25; i++) {
            std::cout << (int) frame->data[i] << " ";
        }
        std::cout << std::endl;

        const char* path = "/tmp/server";
          
        SocketInfo si = WaitForClinet(path);
        TicToc();
        SendFD(si.client_fd, frame->fd);
        TicToc();

        while(true)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
//     {
//         std::cout << getpid() << std::endl;
//         std::cout << frame->fd << std::endl;
//         std::cout << frame->size << std::endl;
// 
//         TicToc();
//         int fd = shm_open("/tucker-mem", O_CREAT | O_RDWR, 0777);
//         if(fd < 0) { HandleError("SHM_OPEN"); }
// 
//         int ret = ftruncate(fd, frame->size);
//         if(ret < 0) { HandleError("FTRUNCATE"); }
// 
//         void* rptr = mmap(NULL, frame->size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
//         if (rptr == MAP_FAILED) { HandleError("MAP FAILED"); }
// 
//         memcpy(rptr, frame->data, frame->size);
//         TicToc();
// 
//         for(size_t i = 0; i < 10; i++) {
//             std::cout << (int) frame->data[i] << " ";
//         }
//         std::cout << std::endl;
// 
//         while(true)
//         {
//             std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//         }
//     }

    // Check that an image is not already being processed
    if(this->is_busy_ == true) {
        return;
    } else {
        this->is_busy_ = true;
    }

    { // Print FPS
        static long long last_time = 0;
        long long now = CurrentTimeMillis();
        if(now - last_time < 1000.0/15.0) { this->is_busy_ = false; return; }
        else {
            std::cout << 1000.0/(now - last_time) << " fps" << std::endl;
            last_time = now;
        }
    }

    if(this->camera_->cameraType() == CAM_FORWARD) {
        const uint32_t w = this->image_width_;
        const uint32_t h = this->image_height_;

        // Convert YUV to RGB
        cv::Mat img = cv::Mat(1.5 * this->image_height_, this->image_width_, CV_8UC1, frame->data);
        cv::cvtColor(img, img, CV_YUV420sp2BGR);
        sensor_msgs::Image::Ptr msg(new sensor_msgs::Image());
        msg->data.resize(3 * w * h);
        memcpy(msg->data.data(), img.ptr(), 3 * w * h);
        msg->encoding = std::string("rgb8");
        msg->width = w;
        msg->height = h;
        msg->step = 3 * w;
        this->it_pub_.publish(msg);
    } else if(this->camera_->cameraType() == CAM_DOWN) {
        const uint32_t w = this->image_width_;
        const uint32_t h = this->image_height_;

        // Downward mono camera
        sensor_msgs::Image::Ptr msg(new sensor_msgs::Image());
        msg->data.resize(w * h);
        memcpy(msg->data.data(), frame->data, w * h);
        msg->encoding = std::string("mono8");
        msg->width = w;
        msg->height = h;
        msg->step = w;
        this->it_pub_.publish(msg);
        std::cout << "This feature not yet implemented! Images can stream, but the bytes are out of order. Please fix this in node.cc!" << std::endl;
    }


    // Finished processing
    this->is_busy_ = false;
    
}

CamConfig Node::LoadCameraConfig() {
    CamConfig config;

    this->nh_->getParam("focus_mode",       config.focusMode);
    this->nh_->getParam("white_balance",    config.whiteBalance);
    this->nh_->getParam("iso",              config.ISO);
    this->nh_->getParam("preview_format",   config.previewFormat);
    this->nh_->getParam("brightness",       config.brightness);
    this->nh_->getParam("sharpness",        config.sharpness);
    this->nh_->getParam("contrast",         config.contrast);

    /* Set resolution size */
    /* Some of these are nonstandard! */
    std::string res;
    this->nh_->getParam("camera_resolution", res);
    if (res == "4k") {
        config.previewSize = CameraSizes::UHDSize();
        this->image_height_ = 2176;
        this->image_width_ = 3840;
    } else if (res == "1080p") {
        config.previewSize = CameraSizes::FHDSize();
        this->image_height_ = 1088;
        this->image_width_ = 1920;
    } else if (res == "720p") {
        config.previewSize = CameraSizes::HDSize();
        this->image_height_ = 736;
        this->image_width_ = 1280;
    } else if (res == "VGA") {
        config.previewSize = CameraSizes::VGASize();
        this->image_height_ = 480;
        this->image_width_ = 640;
    } else {
        ROS_ERROR("Invalid resolution %s!");
        exit(EXIT_FAILURE);
    }

    /* Camera type */
    std::string camera;
    this->nh_->getParam("camera", camera);
    if(camera == "forward") {
        config.cameraId        = 1;
        config.func            = CAM_FORWARD;
    } else if (camera == "down") {
        config.cameraId        = 0;
        config.func            = CAM_DOWN;
    } else {
        ROS_ERROR("Unknown camera!");
        exit(EXIT_FAILURE);
    }

    /* Exposure and gain cannot be set until camera is running */
    config.exposure        = 100;
    config.gain            = 50;

    return config;
}
