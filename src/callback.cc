#include "callback.h"
#include <iostream>
#include <sys/time.h>
#include "sensorParams.h"
#include "GPS.h"
#include "transform.h"

#include <sstream>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iomanip>

/* Sensor params in: sensorParams */
/* GPS solution in: gpsSolution */
void saveImage(const cv::Mat& img, const std::string saveDirectory) {
    /* Setup */
    static int seq = 0;
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << seq++;
    std::string filename = "frame" + ss.str() + ".jpg";

    /* Printing FPS. */
    static long long timeLast = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long timeNow = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    timeLast = timeNow;

    /* Calculate camera pose */
    const Eigen::Vector3d cameraAtt(0, gpsSolution.el - M_PI/6, gpsSolution.az);
    const Eigen::Vector3d rpG(gpsSolution.x, gpsSolution.y, gpsSolution.z);
    const Eigen::Vector3d cameraPos = transformBodyToECEF(
        rpG, 
        Eigen::Vector3d(0,0,0), 
        sensorParams.rcB,
        cameraAtt(0),
        cameraAtt(1),
        cameraAtt(2));

    /* Write camera pose to file */
    std::string data = "" + 
        std::to_string(cameraPos(0)) + " " + 
        std::to_string(cameraPos(1)) + " " + 
        std::to_string(cameraPos(2)) + " " + 
        std::to_string(cameraAtt(0)) + " " + 
        std::to_string(cameraAtt(1)) + " " + 
        std::to_string(cameraAtt(2));

    std::string command = "echo '" + filename + " " + data + "' >> " + saveDirectory + "/image_poses.txt";
    std::system(command.c_str());
    
    /* Saving image */
    // Compress to jpeg
    std::vector<uint8_t> buff;
    std::vector<int> param(2);
    param[0] = cv::IMWRITE_JPEG_QUALITY;
    param[1] = 100;
    cv::imencode(".jpg", img, buff, param);

    // Write
    std::string fullPath = saveDirectory + filename;
    std::ofstream savefile(fullPath.c_str(), std::ios::out | std::ios::binary);
    savefile.write((const char*)&buff[0], buff.size());
    savefile.close();
}
