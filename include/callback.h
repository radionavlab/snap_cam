#pragma once

#include <cv.h>
#include <Eigen/Core>
#include <iostream>

void callback(const cv::Mat& img, 
              const Eigen::Matrix<long double, 3, 1>& camera_position,
              const Eigen::Matrix<long double, 3, 1>& camera_orientation);
