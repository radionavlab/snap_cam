#pragma once

#include <atomic>
#include <vector>

#include <gbx_ros_bridge_msgs/Attitude2D.h>
#include <gbx_ros_bridge_msgs/SingleBaselineRTK.h>
#include <gbx_ros_bridge_msgs/BaseTime.h>


// Precise Position Solution in ECEF coordinates
// Position is in meters
// Pose is in radians
typedef struct {
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> z;
    std::atomic<float> posCov[9];

    std::atomic<double> az;
    std::atomic<double> el;
    std::atomic<float> attCov[4];
} GPSSolution;

extern GPSSolution solution;

/* Roll variance. Set so +-3 sigma is 15 degrees */
const double rollVar = 0.007615435;

void positionMessageHandler(const gbx_ros_bridge_msgs::SingleBaselineRTK msg);
void attitudeMessageHandler(const gbx_ros_bridge_msgs::Attitude2D msg);
