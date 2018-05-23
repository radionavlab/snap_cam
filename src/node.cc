#include "node.h"
#include <iostream>
#include <chrono>
#include <sensor_msgs/Image.h>

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
    // Check that an image is not already being processed
    if(this->is_busy_ == true) {
        std::cout << "busy" << std::endl;
        return;
    } else {
        this->is_busy_ = true;
    }

    { // Print FPS
        static long long last_time = 0;
        std::chrono::milliseconds now = std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
        );

        std::cout << 1000.0/(now.count() - last_time) << " fps" << std::endl;
        last_time = now.count();
    }

    if(this->camera_->cameraType() == CAM_FORWARD) {
        // // Convert YUV to RGB
        // cv::Mat img = cv::Mat(1.5 * this->image_height_, this->image_width_, CV_8UC1, frame->data);
        // cv::cvtColor(img, img, CV_YUV420sp2RGB);
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        // this->it_pub_.publish(msg);

        // Convert YUV420 to YUV422
        // https://github.com/ATLFlight/snap_cam_ros/blob/master/src/snap_cam.cpp
        // Adopted and changed from above link
        // Avoiding the OpenCV conversion above to minimize data copies
        // Removed the CV Libraries from CMakeLists. Will have to put them back in to use opencv.
        sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
        const uint32_t w = this->image_width_;
        const uint32_t h = this->image_height_;

        image->width = w;
        image->height = h;

        const uint32_t image_size_pix = w * h;
        const uint32_t u_offset = image_size_pix;
        image->data.resize(image_size_pix * 2);
        for(size_t i = 0; i * 2 < image_size_pix; i++) {
            const uint32_t row = (i * 2) / w; // full res row
            const uint32_t col = (i * 2) % w; // full res row
            const uint32_t rc_ind = ((row / 2) * (w / 2)) + (col / 2); // half res ind
            const uint32_t u_ind = u_offset + (rc_ind * 2) + 1;
            const uint32_t v_ind = u_offset + (rc_ind * 2);

            // VYUY?
            image->data[(i*4)+0] = frame->data[v_ind];
            image->data[(i*4)+1] = frame->data[i*2]; // Y1
            image->data[(i*4)+2] = frame->data[u_ind];
            image->data[(i*4)+3] = frame->data[(i*2) + 1]; // Y2
        }

        image->encoding = std::string("yuv422");
        image->step = w * 2;

        this->it_pub_.publish(image);
    } else if(this->camera_->cameraType() == CAM_DOWN) {
        // cv::Mat img = cv::Mat(this->image_height_, this->image_width_, CV_8UC1, frame->data);
        // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
        // this->it_pub_.publish(msg);

        // sensor_msgs::Image msg;
        // msg.header       = std_msgs::Header();
        // msg.height       = this->image_height_;
        // msg.width        = this->image_width_;
        // msg.encoding     = "mono8";
        // msg.is_bigendian = true;
        // msg.step         = this->image_width_;
        // msg.data         = std::vector<uint8_t>(frame->data, frame->data + this->image_height_ * msg.step);
        // this->it_pub_.publish(msg);
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
