#include "callback.h"
#include <iostream>
#include <sys/time.h>
#include "sensorParams.h"
#include "GPS.h"

#include <sstream>
#include <opencv2/opencv.hpp>
#include <fstream>

const BalloonInfo processImage(const cv::Mat& img) {
    /* Sensor params in: sensorParams */
    /* GPS solution in: gpsSolution */

    /* 
    * YOUR CODE GOES HERE
    */

    /* Example: Save file to disk */
    static int seq = 0;
    
    // Compress to JPEG
    std::vector<uint8_t> buff; //buffer for jpeg encoding
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 100;
    cv::imencode(".jpg", img, buff, param);

    // Write
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << seq++;
    std::string filename = "frame" + ss.str() + ".jpg";
    std::string fullPath = "/mnt/storage/images/" + filename;
    std::ofstream savefile(fullPath.c_str(), std::ios::out | std::ios::binary);
    savefile.write((const char*)&buff[0], buff.size());
    savefile.close();

    /* Example: Printing info */
    std::cout << "Got image! " << img.size[0] << ", " << img.size[1] << std::endl;
    std::cout << "Camera params: " << sensorParams.f << ", " << sensorParams.k1 << std::endl;
    std::cout << "Antenna Position: " << gpsSolution.x << ", " << gpsSolution.y << ", " << gpsSolution.z << std::endl;

    /* Example: Printing FPS. */
    static long long timeLast = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long timeNow = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    std::cout << "FPS: " << 1000.0 / (timeNow - timeLast) << std::endl;
    timeLast = timeNow;
}
