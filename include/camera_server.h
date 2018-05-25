// Author: Tucker Haydon

#pragma once
#include "types.h"

#include <string>


namespace snapcam {

class CameraServer {
public:
   CameraServer(std::string path); 
   ~CameraServer(); 

   void PublishFrame(const FD& frame_fd, const size_t& data_size, const size_t& width, const size_t& height);


private:
    void ReportError(const std::string& msg);
    void SendFD(const FD& socket_fd, const FD& fd, const std::string& frame_info);

    FD fd_;
    std::string path_;

};

}; // namespace snapcam
