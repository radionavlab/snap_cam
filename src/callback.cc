#include "callback.h"
#include <iostream>
#include <sys/time.h>
#include "sensorParams.h"
#include "GPS.h"

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
    std::cout << "FPS: " << 1000.0 / (timeNow - timeLast) << std::endl;
    timeLast = timeNow;

    /* Write camera pose to file */
    std::string data = "";
    {   // Scoping block
        std::string command = "echo '# IMAGENAME X Y Z POSCOV EL AZ ELSIGMA AZSIGMA ATTCOV' >> " + saveDirectory + "/image_data_raw.txt";
        std::system(command.c_str());
    }
    
    // Write camera position
    data = data + " " + std::to_string(gpsSolution.x);
    data = data + " " + std::to_string(gpsSolution.y);
    data = data + " " + std::to_string(gpsSolution.z);

    for(int i=0; i < 6; i++) {
        data = data + " " + std::to_string(gpsSolution.posCov[i]);
    }

    // Write camera orientation
    data = data + " " + std::to_string(gpsSolution.el);
    data = data + " " + std::to_string(gpsSolution.az);

    data = data + " " + std::to_string(gpsSolution.elSigma);
    data = data + " " + std::to_string(gpsSolution.azSigma);
 
    for(int i=0; i < 6; i++) {
        data = data + " " + std::to_string(gpsSolution.attCov[i]);
    }

    std::string command = "echo '" + filename + data + "' >> " + saveDirectory + "/image_data_raw.txt";
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
