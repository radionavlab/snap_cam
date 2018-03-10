#pragma once
#include <Eigen/Core>

typedef struct {
    /* Position of the camera with respect to body origin in body frame */
    Eigen::Vector3d rcB;

    /* Position of primary with respect to body origin in body frame */
    Eigen::Vector3d rpB;

    /* Position of reference antenna with respect to ECEF origin in ECEF frame */
    Eigen::Vector3d rrG;

    /* Position of inertial origin with respect to ECEF origin in ECEF frame */
    Eigen::Vector3d riG;

    /* Focal length of camera in pixels */
    double f;

    /* Camera intrinsics */
    double k1;

    /* Image size */
    uint16_t imageWidth;
    uint16_t imageHeight;


} SensorParams;

extern SensorParams sensorParams;
