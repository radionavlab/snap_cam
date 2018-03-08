#pragma once

#include <cv.h>
#include <Eigen/Core>

const Eigen::Vector3i calcBalloonPosition(const cv::Mat& img);
