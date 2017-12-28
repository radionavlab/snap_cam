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


// Precise Position Solution in ECEF coordinates
// Position is in meters
// Pose is in radians
struct PPSolution {
    std::atomic<double> roverX;
    std::atomic<double> roverY;
    std::atomic<double> roverZ;

    std::atomic<double> azimuth;
    std::atomic<double> elevation;

    std::atomic<long> week;
    std::atomic<long> secondsOfWeek;
    std::atomic<double> fractionOfSecond;
} solution;

std::atomic<bool> camera_busy{false};

/* Camera Publishers */
ros::Publisher camera_image_pub;
ros::Publisher camera_position_pub;

/* Camera resolution */
int camera_width, camera_height;
int camera_number;

/* Camera position */
Eigen::Matrix<long double, 3, 1> camera_body_position;

/* Save Directory for images */
std::string save_directory;

/* Image callback options */
bool publish_image_option{false};
bool save_image_option{false};

std::shared_ptr<CamConfig> init_down_camera_config();
std::shared_ptr<CamConfig> init_front_camera_config(ros::NodeHandle& nh);

void positionMessageHandler(const gbx_ros_bridge_msgs::SingleBaselineRTK msg);
void attitudeMessageHandler(const gbx_ros_bridge_msgs::Attitude2D msg);

void down_callback(ICameraFrame *frame);

void compress_image(std::vector<uint8_t>& buff, cv::Mat& img, const int compression_quality);

void create_compressed_image_message(sensor_msgs::CompressedImage& image_msg, std::vector<uint8_t>& buff, const int seq);
void calc_camera_position(Eigen::Matrix<long double, 3, 1>& camera_position);
void save_camera_position(Eigen::Matrix<long double, 3, 1>& camera_position, const int seq);
void save_image(std::vector<uint8_t>& buff, const int seq);
void create_camera_position_message(geometry_msgs::Pose& pose_msg, Eigen::Matrix<long double, 3, 1>& camera_position);
void print_fps();
void frame_handler(ICameraFrame *frame);
void set_callback_mode(ros::NodeHandle& nh);
void create_images_directory(ros::NodeHandle& nh);
void read_camera_position(ros::NodeHandle& nh);
