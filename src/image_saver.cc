// Author: Tucker Haydon

#include "image_saver.h"

#include <iomanip>
#include <sys/stat.h>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <sstream>

ImageSaver::ImageSaver(const std::string& root_directory_path,
                       std::shared_ptr<MetadataLogger> metadata_logger_ptr) {
    this->save_directory_path_ = root_directory_path;
    this->MakeSaveDirectory();
    this->metadata_logger_ptr_ = metadata_logger_ptr;
};

std::string ImageSaver::NextImageFileName() {
    static int seq = 0;
    std::stringstream ss;
    ss << std::setw(5) << std::setfill('0') << seq++;
    std::string filename = "frame" + ss.str() + ".yuv";
    return filename;
}; 

void ImageSaver::SaveImage(camera::ICameraFrame *frame) { 
    // Calc and report FPS
    static long long last_time_ms = 0;
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long now_time_ms = tp.tv_sec * 1000 + tp.tv_usec / 1000;
    double fps = 1000.0 / (now_time_ms - last_time_ms);
    last_time_ms = now_time_ms;
    std::cout << std::setw(10) << std::setprecision(3) << fps << " fps" << '\r' << std::flush;


    // Log metadata
    const std::string filename = this->NextImageFileName();
    this->metadata_logger_ptr_->LogMetadata(filename);
 
    // Write image data to disk
    // std::ios_base::sync_with_stdio(false);
    std::fstream img_file(this->save_directory_path_ + "/" + filename, std::ios::out | std::ios::binary);
    img_file.write((char*)frame->data, frame->size);
    img_file.close();
};

void ImageSaver::MakeSaveDirectory() {
  /* Make a directory for all of the images */
  time_t now = time(0);
  tm *ltm = localtime(&now);
  std::string year = std::to_string(1900 + ltm->tm_year);
  std::string month = std::to_string(1 + ltm->tm_mon);
  std::string day = std::to_string(ltm->tm_mday);
  std::string hour = std::to_string(ltm->tm_hour);
  std::string min = std::to_string(ltm->tm_min);
  std::string sec = std::to_string(ltm->tm_sec);
  std::string dir = year + "-" + month + "-" + day + "-" + hour + "-" + min + "-" + sec + "/";
  this->save_directory_path_ = this->save_directory_path_ + dir;
  const int err = mkdir(this->save_directory_path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if(err == -1){
      std::cout << "Could not make save directory!" << std::endl;
      exit(EXIT_FAILURE);
  }
};

