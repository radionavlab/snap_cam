#pragma once

#include <cv.h>
#include <Eigen/Core>
#include <iostream>

void callback(const cv::Mat& img, 
              const Eigen::Matrix<long double, 3, 1>& camera_position,
              const Eigen::Matrix<long double, 3, 3>& camera_position_covariance,
              const Eigen::Matrix<long double, 3, 1>& camera_orientation,
              const Eigen::Matrix<long double, 3, 3>& camera_orientation_covariance,
              const double f,
              const double k1);
