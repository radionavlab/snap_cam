#pragma once

#include <atomic>
#include <vector>

#include <mg_msgs/Attitude2D.h>
#include <mg_msgs/SingleBaselineRTK.h>
#include <mg_msgs/BaseTime.h>


// Position of primary antenna with respect to reference in ECEF
// Position is in meters
// Pose is in radians
typedef struct {
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> z;
    std::atomic<float> posCov[9];

    std::atomic<double> az;
    std::atomic<double> el;
    std::atomic<float> attCov[9];
} GPSSolution;

extern GPSSolution gpsSolution;

/* Roll variance. Set so +-3 sigma is 15 degrees */
const double rollVar = 0.007615435;

void positionMessageHandler(const mg_msgs::SingleBaselineRTK msg);
void attitudeMessageHandler(const mg_msgs::Attitude2D msg);
