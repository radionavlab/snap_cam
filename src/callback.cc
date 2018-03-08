#include "callback.h"
#include <iostream>
#include <sys/time.h>
#include "sensorParams.h"

const Eigen::Vector3i calcBalloonPosition(const cv::Mat& img) {
    /* Sensor params with camera intrinsics and focal length */
    const SensorParams params = sensorParams;

    /* 
    * YOUR CODE GOES HERE
    */

    // Example: Printing info
    std::cout << "Got image! " << img.size[0] << ", " << img.size[1] << std::endl;
    std::cout << "Camera params: " << params.f << ", " << params.k1 << std::endl;

    // Example: Printing FPS.
    static long long timeLast = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long timeNow = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    std::cout << "FPS: " << 1000 / (timeNow - timeLast) << std::endl;
    timeLast = timeNow;
}
