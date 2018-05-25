#pragma once

#include <atomic>
#include <memory>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "snapcam.h"
#include "camera_server.h"

class Node{
public:
    Node(int argc, char** argv);
    void FrameHandler(camera::ICameraFrame* frame);
    void Start();

private:
    CamConfig LoadCameraConfig();

    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<SnapCam> camera_;
    std::shared_ptr<snapcam::CameraServer> camera_server_;

    std::atomic<bool> is_busy_{false};
    int image_width_;
    int image_height_;

};
