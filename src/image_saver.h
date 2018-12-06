// Author: Tucker Haydon

#ifndef IMAGE_SAVER_IMAGE_SAVER_H
#define IMAGE_SAVER_IMAGE_SAVER_H

#include <string>
#include <atomic>
#include <memory>
#include <camera.h>
#include "metadata_logger.h"

class ImageSaver {
  private:
    std::string save_directory_path_;
    std::shared_ptr<MetadataLogger> metadata_logger_ptr_;
   
   /*
    * Generate the next image name.
    */ 
    std::string NextImageFileName(); 

    /*
     * Make the directory where images are saved
     */
    void MakeSaveDirectory();

  public: 
    /*
     * Constructor.
     */
    ImageSaver(const std::string& save_directory_path,
               std::shared_ptr<MetadataLogger> metadata_logger_ptr);

    /*
     * Save an image to disk.
     */
    void SaveImage(camera::ICameraFrame *frame); 
};

#endif
