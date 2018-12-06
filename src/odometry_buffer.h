//Author: Tucker Haydon
#ifndef ODOMETRY_BUFFER_H
#define ODOMETRY_BUFFER_H

#include <mutex>
#include <nav_msgs/Odometry.h>
#include <iostream>

class OdometryBuffer {
  private:
    nav_msgs::Odometry odometry_;
    static std::mutex mtx_;

  public:
    OdometryBuffer() {};

    nav_msgs::Odometry GetOdometryMsg() {
      std::lock_guard<std::mutex> guard(OdometryBuffer::mtx_);
      return odometry_; 
    };

    void SetOdometryMsg(const nav_msgs::Odometry& odometry) {
      std::lock_guard<std::mutex> guard(OdometryBuffer::mtx_);
      this->odometry_ = odometry;
    };
};

extern OdometryBuffer odometry_buffer;

#endif
