#pragma once 

#include <atomic>
#include <ros/ros.h>
#include <Eigen/Core>

#include <geometry_msgs/Point.h>
#include <mg_msgs/BalloonInfo.h>

#include "snapcam.h"
#include "callback.h"
#include "sensorParams.h"
#include "GPS.h"


std::atomic<bool> isCameraBusy{false};
std::shared_ptr<SnapCam> cam;

/* ROS topics */
std::string attitudeTopic;
std::string positionTopic;
std::string balloonTopic;

/* ROS publishers */
ros::Publisher balloonInfoPublisher;

/* Function stubs */
std::shared_ptr<CamConfig> initFrontCameraConfig(ros::NodeHandle& nh);
void frameHandler(ICameraFrame *frame);
void loadParams(const ros::NodeHandle& nh);

