#include "node.h"
#include "odometry_message_handler.h"

#include <string>

Node::Node(int argc, char** argv) {
    ros::init(argc, argv, "image_saver", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    this->nh_ = std::make_shared<ros::NodeHandle>("~");
    
    std::string odometry_topic;
    this->nh_->getParam("odometry_topic", odometry_topic);
    this->odometry_sub_ = this->nh_->subscribe(odometry_topic, 1, OdometryMessageHandler);
}

void Node::Start() {
    ros::Rate r(100);
    while(this->nh_->ok()) {
        ros::spinOnce();
        r.sleep();
    }
}

void Node::Stop() {
    ros::shutdown();
}
