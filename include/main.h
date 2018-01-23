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

#define DEFAULT_EXPOSURE_VALUE  250
#define MIN_EXPOSURE_VALUE 1
#define MAX_EXPOSURE_VALUE 511
#define DEFAULT_GAIN_VALUE  50
#define MIN_GAIN_VALUE 0
#define MAX_GAIN_VALUE 150

#define EXPOSURE_CHANGE_THRESHOLD 10.0f
#define GAIN_CHANGE_THRESHOLD 5.0f
#define MSV_TARGET 5.0f

#define EXPOSURE_P 30.0f
#define EXPOSURE_I 0.1f
#define EXPOSURE_D 0.1f

#define GAIN_P 15.0f
#define GAIN_I 0.05f
#define GAIN_D 0.05f

#define HISTOGRAM_MASK_SIZE 128

// Precise Position Solution in ECEF coordinates
// Position is in meters
// Pose is in radians
struct PPSolution {
    std::atomic<double> roverX;
    std::atomic<double> roverY;
    std::atomic<double> roverZ;

    std::atomic<double> azimuth;
    std::atomic<double> elevation;
} solution;

std::atomic<bool> camera_busy{false};
std::shared_ptr<SnapCam> cam;

/* Camera position */
Eigen::Matrix<long double, 3, 1> camera_body_position;

/* Camera info */
double camera_width, camera_height;

std::shared_ptr<CamConfig> init_front_camera_config(ros::NodeHandle& nh);

void positionMessageHandler(const gbx_ros_bridge_msgs::SingleBaselineRTK msg);
void attitudeMessageHandler(const gbx_ros_bridge_msgs::Attitude2D msg);
void calc_camera_position(Eigen::Matrix<long double, 3, 1>& camera_position);
void frame_handler(ICameraFrame *frame);
void read_camera_position(ros::NodeHandle& nh);
