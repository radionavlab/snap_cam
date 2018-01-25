#include "callback.h"
#include <sys/time.h>

void callback(const cv::Mat& img, 
              const Eigen::Matrix<long double, 3, 1>& camera_position,
              const Eigen::Matrix<long double, 3, 3>& camera_position_covariance,
              const Eigen::Matrix<long double, 3, 1>& camera_orientation,
              const Eigen::Matrix<long double, 3, 3>& camera_orientation_covariance,
              const double f,
              const double k1) {
        /* 
        * YOUR CODE GOES HERE
        */

        // Example: Printing info
        std::cout << "Got image! " << img.size[0] << ", " << img.size[1] << std::endl;
        std::cout << "Camera params: " << f << ", " << k1 << std::endl;

        // Example: Printing FPS.
        static long long last_time = 0;
        struct timeval tp;
        gettimeofday(&tp, NULL);
        long long now_time = tp.tv_sec * 1000 + tp.tv_usec / 1000;
        std::cout << "FPS: " << 1000 / (now_time - last_time) << std::endl;
        last_time = now_time;

}
