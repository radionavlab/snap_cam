/*
*      Author: Tucker Haydon
*/
#pragma once 

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
#include <atomic>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <gbx_ros_bridge_msgs/Attitude2D.h>
#include <gbx_ros_bridge_msgs/SingleBaselineRTK.h>

#include "snapcam.h"
#include "coordinate.h"
#include "Eigen/Core"
#include "callback.h"

// Precise Position Solution in ECEF coordinates
// Position is in meters
// Pose is in radians
struct PPSolution {
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> z;
    std::atomic<float> pos_cov[9];

    std::atomic<double> az;
    std::atomic<double> el;
    std::atomic<float> att_cov[4];
} solution;

std::atomic<bool> camera_busy{false};
std::shared_ptr<SnapCam> cam;

/* Camera position */
Eigen::Matrix<long double, 3, 1> camera_body_position;

/* Camera Instrinsics */
const double f = 1656.06;
const double k1 = -0.0269765;

/* Roll variance. Set so +-3 sigma is 15 degrees */
const double roll_var = 0.007615435;

/* Camera info */
double camera_width, camera_height;

/* Function stubs */
std::shared_ptr<CamConfig> init_front_camera_config(ros::NodeHandle& nh);
void positionMessageHandler(const gbx_ros_bridge_msgs::SingleBaselineRTK msg);
void attitudeMessageHandler(const gbx_ros_bridge_msgs::Attitude2D msg);
void calc_camera_position(Eigen::Matrix<long double, 3, 1>& camera_position);
void frame_handler(ICameraFrame *frame);
void read_camera_position(ros::NodeHandle& nh);
