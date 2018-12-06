// Author: Tucker Haydon

#ifndef IMAGE_SAVER_METADATA_LOGGER_H
#define IMAGE_SAVER_METADATA_LOGGER_H

#include <string>
#include <fstream>

class MetadataLogger {
  private:
    std::string log_file_name_;
    std::string log_file_path_;
    std::string log_directory_path_;
    std::ofstream* output_file_ptr_;

    /*
     * Create a log file and append the header lines to it
     */
    void CreateLogFile();

  public:
    /*
     * Constructor
     */
    MetadataLogger(const std::string& log_directory_path, const std::string& log_file_name);

    /*
     * Destructor. Ensures proper destruction of resource handles.
     */
    ~MetadataLogger();

    /* 
     * Log the current metadata. Appends a row to the log file beginning with
     * the row name parameter.
     */
    void LogMetadata(const std::string& row_name);
};
#endif
