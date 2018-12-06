// Author: Tucker Haydon

#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <thread>
#include <iostream>
#include <memory>
#include <functional>

#include "node.h"
#include "image_saver.h"
#include "snap_cam.h"

namespace{
  std::shared_ptr<Node> node_ptr;
}

void WaitExit() {
    std::cin.tie(nullptr);
    do {
        std::cout << "Press 'q' to quit." << std::endl << std::endl;
    } while (std::cin.get() != 'q');
    node_ptr->Stop();
}

int main(int argc, char **argv) {
    static const std::string save_directory_path = "/mnt/storage/images/";
    auto metadata_logger_ptr = std::make_shared<MetadataLogger>(save_directory_path, "metadata.log");
    ImageSaver is(save_directory_path, metadata_logger_ptr);
    SnapCam snap_cam;
    snap_cam.SetListener(std::bind(&ImageSaver::SaveImage, is, std::placeholders::_1));

    node_ptr = std::make_shared<Node>(argc, argv);

    std::thread node_thread(&Node::Start, node_ptr.get());
    std::thread exit_thread(WaitExit);

    node_thread.join();
    exit_thread.join();

    return EXIT_SUCCESS;
}

