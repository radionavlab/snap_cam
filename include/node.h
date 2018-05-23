#pragma once

#include <atomic>
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <sensor_msgs/Image.h>

#include "snapcam.h"

class Node{
public:
    Node(int argc, char** argv);
    void FrameHandler(camera::ICameraFrame* frame);
    void Start();

private:
    CamConfig LoadCameraConfig();

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    std::shared_ptr<SnapCam> camera_;
    image_transport::Publisher it_pub_;

    std::atomic<bool> is_busy_{false};
    int image_width_;
    int image_height_;

};
