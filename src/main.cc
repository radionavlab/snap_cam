// Author: Tucker Haydon

#include <cstdlib>
#include <unistd.h>
#include <signal.h>
#include <string>
#include <thread>
#include <iostream>
#include <memory>
#include <functional>
#include <sys/stat.h>
#include <csignal>

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

std::string MakeSaveDirectory(const std::string& parent_directory) {
  /* Make a directory for all of the images */
  time_t now = time(0);
  tm *ltm = localtime(&now);
  std::string year = std::to_string(1900 + ltm->tm_year);
  std::string month = std::to_string(1 + ltm->tm_mon);
  std::string day = std::to_string(ltm->tm_mday);
  std::string hour = std::to_string(ltm->tm_hour);
  std::string min = std::to_string(ltm->tm_min);
  std::string sec = std::to_string(ltm->tm_sec);
  std::string name = year + "-" + month + "-" + day + "-" + hour + "-" + min + "-" + sec + "/";
  std::string full_path = parent_directory + name;
  const int err = mkdir(full_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if(err == -1){
      std::cout << "Could not make save directory!" << std::endl;
      exit(EXIT_FAILURE);
  }
  return full_path;
};

int main(int argc, char **argv) {

  // Variables
  static const std::string metadata_filename = "metadata.log";
  static const std::string parent_path = "/mnt/storage/images/";
  static const std::string save_directory_path = MakeSaveDirectory(parent_path);

  // Create logger
  std::shared_ptr<MetadataLogger> metadata_logger_ptr = std::make_shared<MetadataLogger>(save_directory_path, metadata_filename);

  // Create image saver
  ImageSaver is(save_directory_path, metadata_logger_ptr);

  // Create ros node
  node_ptr = std::make_shared<Node>(argc, argv);

  // Create snap cam
  SnapCam snap_cam;
  snap_cam.SetListener(std::bind(&ImageSaver::SaveImage, is, std::placeholders::_1));

  // Start threads
  std::thread node_thread(&Node::Start, node_ptr.get());
  std::thread exit_thread(WaitExit);
  snap_cam.Start();

  // Wait for stop
  node_thread.join();
  exit_thread.join();
  snap_cam.Stop();

  // Exit
  return EXIT_SUCCESS;
}

