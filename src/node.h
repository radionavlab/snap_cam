// Author: Tucker Haydon
#ifndef IMAGE_SAVER_NODE_H
#define IMAGE_SAVER_NODE_H

#include <memory>
#include <ros/ros.h>

class Node {
public:
    Node(int argc, char** argv);
    void Start();
    void Stop();

private:
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber odometry_sub_;
};

#endif
