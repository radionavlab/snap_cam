#pragma once

#include <cv.h>
#include <Eigen/Core>

const Eigen::Vector3d calcBalloonPosition(const cv::Mat& img);
