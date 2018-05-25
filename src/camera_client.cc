#include <iostream>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/un.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <chrono>
#include <vector>
#include <sstream>
#define PATH "/tmp/camera_server"

long long CurrentTimeMillis() {
    return std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

void TicToc() {
    static long long previous = 0L;
    static bool tic = false;

    long long now = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    if(tic) {
       tic = false;
       std::cout << "Time Elapsed: " << now - previous << " ms" << std::endl; 
    } else {
        tic = true;
        previous = now;
    }
}

void handle_error(const char* msg) {
    std::cout << msg << std::endl;
    exit(EXIT_FAILURE);
}

static int receivefd(int socket) {
    struct msghdr msg = {0};

    char m_buffer[256];
    struct iovec io = { .iov_base = m_buffer, .iov_len = sizeof(m_buffer) };
    msg.msg_iov = &io;
    msg.msg_iovlen = 1;

    char c_buffer[256];
    msg.msg_control = c_buffer;
    msg.msg_controllen = sizeof(c_buffer);

    if (recvmsg(socket, &msg, 0) < 0) {
        std::cout << "Failed to receive message" << std::endl;
    }

    std::string frame_info = std::string((char*)io.iov_base);
    std::vector<int> frame_info_vec;
    std::stringstream ss(frame_info);
    int i;
    while (ss >> i)
    {
        frame_info_vec.push_back(i);

        if (ss.peek() == ',')
            ss.ignore();
    }
    
    const int data_size = frame_info_vec[0];
    const int width = frame_info_vec[1];
    const int height = frame_info_vec[2];


    struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msg);
    unsigned char * data = CMSG_DATA(cmsg);

    std::cout << "About to extract fd" << std::endl;
    int fd = *((int*) data);
    std::cout << "Extracted fd: " << fd << std::endl;

    return fd;
}

int main(int argc, char** argv) {
    int sock = 0;
    struct sockaddr_un server_addr;

    // Creating socket file descriptor
    if ((sock = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
        handle_error("socket");
    }
  
    // Connect to unix port server
    memset(&server_addr, '0', sizeof(struct sockaddr_un)); 
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, PATH, sizeof(server_addr.sun_path) -1); 
  
    int length = 12546048;
    uint8_t* buff = (uint8_t*) malloc(length);
    TicToc();
    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr_un)) < 0) {
        handle_error("connect");
    }

    int fd = receivefd(sock);
    uint8_t *ptr = (uint8_t *) mmap(NULL, length, PROT_READ, MAP_SHARED, fd, 0);
    memcpy(buff, ptr, length);
    TicToc();
    close(fd);

    // std::cout << "PID: " << getpid() << std::endl;
    // std::cout << fd << std::endl;
    for(size_t i = 0; i < 25; i++) {
        std::cout << (int) buff[i] << " ";
    }
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
