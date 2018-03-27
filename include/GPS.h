#pragma once

#include <atomic>

#include <mg_msgs/Attitude2D.h>
#include <mg_msgs/SingleBaselineRTK.h>


// Position of primary antenna with respect to reference in ECEF
// Position is in meters
// Pose is in radians
typedef struct {
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> z;
    std::atomic<float> posCov[6];
    std::atomic<double> az;
    std::atomic<double> el;
    std::atomic<double> elSigma;
    std::atomic<double> azSigma;
    std::atomic<float> attCov[6];
} GPSSolution;

extern GPSSolution gpsSolution;

void positionMessageHandler(const mg_msgs::SingleBaselineRTK msg);
void attitudeMessageHandler(const mg_msgs::Attitude2D msg);
