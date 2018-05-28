// Author: Tucker Haydon

#pragma once

#include "types.h"

#include <string>

namespace snapcam {

class CameraClient {
public:
    CameraClient(const std::string& path);
    FrameData RequestFrame(); 

private:
    std::string path_;
    FD server_fd_;

    void HandleError(const char* msg);
    void Connect();
};

}; // namespace snapcam
