#pragma once 

#include <iostream>
#include <cstdlib>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include <sys/un.h>

typedef struct {
    int server_fd;
    int client_fd;
} SocketInfo;


void HandleError(const char* msg) {
    std::cout << msg << std::endl;
    std::cout << "Errno: " << errno << std::endl;
    exit(EXIT_FAILURE);
}

static void SendFD(int socket, int fd) {
    struct msghdr msg = { 0 };
    char buf[CMSG_SPACE(sizeof(fd))];
    memset(buf, '\0', sizeof(buf));
    struct iovec io = { .iov_base = (void*)"TUCKER HAYDON", .iov_len = 13 }; // Dummy message

    msg.msg_iov = &io;
    msg.msg_iovlen = 1;
    msg.msg_control = buf;
    msg.msg_controllen = sizeof(buf);

    struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(sizeof(fd));

    *((int *) CMSG_DATA(cmsg)) = fd;

    msg.msg_controllen = cmsg->cmsg_len;

    if (sendmsg(socket, &msg, 0) < 0) {
        std::cout << "Failed to send message" << std::endl;
    }
}

SocketInfo WaitForClinet(const char* path) {
    int server_fd, client_fd;
    struct sockaddr_un server_addr, client_addr;
    const int addrlen = sizeof(server_addr);
    socklen_t client_addr_size = sizeof(struct sockaddr_un);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == 0) {
            HandleError("socket");
    } 

    // Clear and fill server_addr
    memset(&server_addr, 0, sizeof(struct sockaddr_un));
    server_addr.sun_family = AF_UNIX;
    strncpy(server_addr.sun_path, path, sizeof(server_addr.sun_path) - 1);

    // Allow socket to be reused
    bool reuse = true;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(int));

    // Bind to a socket server
    if (bind(server_fd, (struct sockaddr *) &server_addr, sizeof(struct sockaddr_un)) < 0) {
        if(errno == EADDRINUSE){ HandleError("in use"); } 
        else                   { HandleError("bind"); }
    }

    // Listen for a client
    if (listen(server_fd, 1) < 0) { HandleError("listen"); }

    // Accept a client
    client_fd = accept(server_fd, (struct sockaddr *) &client_addr, &client_addr_size);
    if (client_fd == -1) { HandleError("accept"); }

    // Return the client's file descriptor
    return SocketInfo{server_fd, client_fd};
}

