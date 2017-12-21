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

#include "SnapCam.h"
#include "coordinate.h"
#include "Eigen/Core"

std::atomic<bool> is_writing{false};

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

/* Camera parameters */
CamConfig cfg;

/* Camera Publishers */
ros::Publisher camera_image_pub;
ros::Publisher camera_position_pub;

/* Camera resolution */
int height;
int width;

/* Camera position */
Eigen::Matrix<long double, 3, 1> camera_position;

/* Save Directory for images */
std::string save_directory;

void writer(ICameraFrame *frame);
void publisher(ICameraFrame *frame);

void attitudeMessageHandler(const gbx_ros_bridge_msgs::Attitude2D msg) {
    const double rx = msg.rx;
    const double ry = msg.ry;
    const double rz = msg.rz;
    const double rxRov = msg.rxRov;
    const double ryRov = msg.ryRov;
    const double rzRov = msg.rzRov;
    const gbx_ros_bridge_msgs::BaseTime tSolution = msg.tSolution;
    const double deltRSec = msg.deltRSec;
    const std::vector<float> P = msg.P;
    const uint32_t nCov = msg.nCov;
    const double azAngle = msg.azAngle;
    const double elAngle = msg.elAngle;
    const double azSigma = msg.azAngle;
    const double elSigma = msg.elSigma;
    const double testStat = msg.testStat;
    const uint8_t numDD = msg.numDD;
    const uint8_t bitfield = msg.bitfield;

    solution.elevation = elAngle;
    solution.azimuth = azAngle;
}

void positionMessageHandler(const gbx_ros_bridge_msgs::SingleBaselineRTK msg) {
    const double rx = msg.rx;
    const double ry = msg.ry;
    const double rz = msg.rz;
    const double rxRov = msg.rxRov;
    const double ryRov = msg.ryRov;
    const double rzRov = msg.rzRov;
    const gbx_ros_bridge_msgs::BaseTime tSolution = msg.tSolution;
    const double deltRSec = msg.deltRSec;
    const std::vector<float> P = msg.P;
    const uint32_t nCov = msg.nCov;
    const double testStat = msg.testStat;
    const double ageOfReferenceData = msg.ageOfReferenceData;
    const uint8_t numDD = msg.numDD;
    const uint8_t bitfield = msg.bitfield;

    solution.roverX = rxRov;
    solution.roverY = ryRov;
    solution.roverZ = rzRov;
}

