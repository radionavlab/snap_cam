// Author: Tucker Haydon

#pragma once

#include <string>
#include <vector>
#include <sstream>

namespace snapcam {
    typedef int FD;

    typedef struct {
        uint32_t data_size;
        uint32_t width;
        uint32_t height;

        std::string Serialize() {
            return ""
                + std::to_string(data_size) + ","
                + std::to_string(width) + ","
                + std::to_string(height);
        };

        void Deserialize(const std::string& serial_data) {
            std::stringstream ss(serial_data);
            std::vector<uint32_t> frame_info_vec;
            int i;
            while (ss >> i) {
                frame_info_vec.push_back(i);

                if (ss.peek() == ',')
                    ss.ignore();
            }
            
            data_size = frame_info_vec[0];
            width     = frame_info_vec[1];
            height    = frame_info_vec[2];
        };

    } FrameMetaData;

    typedef struct {
        FrameMetaData meta_data;
        uint8_t* data;
    } FrameData;

}; // namespace snapcam
