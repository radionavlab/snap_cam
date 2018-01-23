#include "callback.h"

void callback(const cv::Mat& img,
              const Eigen::Matrix<long double, 3, 1>& camera_position,
              const Eigen::Matrix<long double, 3, 1>& camera_orientation) {
        /* 
        * YOUR CODE GOES HERE
        */
        std::cout << "Got image! " << img.size[0] << ", " << img.size[1] << std::endl;
}
