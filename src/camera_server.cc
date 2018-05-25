// Author: Tucker Haydon

#include "camera_server.h"

#include <iostream>
#include <cstdlib>
#include <fcntl.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <errno.h>
#include <sys/un.h>
#include <thread>
#include <chrono>


namespace snapcam {

CameraServer::CameraServer(std::string path) 
    : path_(path) {
    struct sockaddr_un server_address;
    constexpr size_t ADDRESS_SIZE = sizeof(struct sockaddr_un);

    // Create a server socket as a UNIX TCP socket.
    if ((this->fd_ = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0)) <= 0) { 
            this->ReportError("Server socket creation"); 
    } 

    // Clear and fill server_address
    memset(&server_address, 0, ADDRESS_SIZE);
    server_address.sun_family = AF_UNIX;
    strncpy(server_address.sun_path, this->path_.c_str(), sizeof(server_address.sun_path) - 1);

    // Socket options
    constexpr bool OPT = true;
    constexpr size_t OPT_SIZE = sizeof(bool);
    setsockopt(this->fd_, SOL_SOCKET, SO_REUSEADDR, &OPT, OPT_SIZE);

    // Bind the socket
    if (bind(this->fd_, (struct sockaddr *) &server_address, ADDRESS_SIZE) < 0) {
        this->ReportError("Error binding server socket."); 
    }

    // Start the socket listening
    #define MAX_CLIENTS 128
    if (listen(this->fd_, MAX_CLIENTS) < 0) { 
        this->ReportError("listen"); 
    }
};

CameraServer::~CameraServer() {
    close(this->fd_);
    unlink(this->path_.c_str());

}

void CameraServer::PublishFrame(const FD& frame_fd, const size_t& data_size, const size_t& width, const size_t& height) {
    struct sockaddr_un client_address;
    socklen_t ADDRESS_SIZE = sizeof(struct sockaddr_un);

    while(true) {
        // Accept a client
        FD client_fd = accept(this->fd_, (struct sockaddr *) &client_address, &ADDRESS_SIZE);
        if (client_fd < 0) { 
                // If no queued clients, break
                if(errno == EWOULDBLOCK) {
                    break;
                } else {
                    this->ReportError("Error connecting client");
                }
        }
        std::string frame_info = "" 
            +       std::to_string(data_size) 
            + "," + std::to_string(width) 
            + "," + std::to_string(height);
        SendFD(client_fd, frame_fd, frame_info);
        close(client_fd);
    }
  
    // Permit subscribers to use the frame for 100 ms 
    // TODO this prevents the client from reading the information
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));  
}

void CameraServer::SendFD(const FD& client_fd, const FD& frame_fd, const std::string& frame_info) {

    struct msghdr msg = { 0 };
    char buf[CMSG_SPACE(sizeof(frame_fd))];
    memset(buf, '\0', sizeof(buf));
    struct iovec io = { .iov_base = (void*)frame_info.c_str(), .iov_len = frame_info.size() + 1 }; 

    msg.msg_iov = &io;
    msg.msg_iovlen = 1;
    msg.msg_control = buf;
    msg.msg_controllen = sizeof(buf);

    struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(frame_fd));

    *((int *) CMSG_DATA(cmsg)) = frame_fd;

    msg.msg_controllen = cmsg->cmsg_len;

    if (sendmsg(client_fd, &msg, 0) < 0) {
        this->ReportError("Failed to send FD.");
    }
}


void CameraServer::ReportError(const std::string& msg) {
    std::cout << msg << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    exit(EXIT_FAILURE);
}

}; // namespace snapcam
