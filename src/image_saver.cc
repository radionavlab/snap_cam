// Author: Tucker Haydon

#include "image_saver.h"

#include <iomanip>
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <sstream>

ImageSaver::ImageSaver(const std::string& root_directory_path,
                       std::shared_ptr<MetadataLogger> metadata_logger_ptr) {
    this->save_directory_path_ = root_directory_path;
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
    // Log metadata
    const std::string filename = this->NextImageFileName();
    this->metadata_logger_ptr_->LogMetadata(filename);
 
    // Write image data to disk
    std::fstream img_file(this->save_directory_path_ + "/" + filename, std::ios::out | std::ios::binary);
    img_file.write((char*)frame->data, frame->size);
    img_file.close();

    // Report image saved
    std::cout << std::setw(30) << std::left << filename + " saved!"  << '\r' << std::flush;
};


