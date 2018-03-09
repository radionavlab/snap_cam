#pragma once 

#include <atomic>
#include <ros/ros.h>
#include <Eigen/Core>
#include <sys/stat.h>

#include <geometry_msgs/Point.h>
#include <mg_msgs/BalloonInfo.h>

#include "snapcam.h"
#include "callback.h"
#include "sensorParams.h"
#include "GPS.h"


/* Globals */
std::atomic<bool> isCameraBusy{false};
std::shared_ptr<SnapCam> cam;
std::string saveDirectory;

/* ROS topics */
std::string attitudeTopic;
std::string positionTopic;
std::string balloonTopic;

/* Function stubs */
std::shared_ptr<CamConfig> initFrontCameraConfig(ros::NodeHandle& nh);
void frameHandler(ICameraFrame *frame);
void loadParams(const ros::NodeHandle& nh);
void makeImageDirectory();

