#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry> 
#include <cmath>
#include "sensorParams.h"

/**
* This function was adapted from gss/shared/core/navtoolbox.
* X, Y, Z in meters in ECEF coordinate frame
* Lat, Lon in radians
* Alt in meters
*/
Eigen::Vector3d ECEFToLatLonAlt(
    const Eigen::Vector3d pos
    ) {

        const long double X = pos(0);
        const long double Y = pos(1);
        const long double Z = pos(2);

        // Meters
        const long double AA_WGS84 = 6378137.00000;     

        // eccentricity squared
        const long double esquare_WGS84 = 0.00669437998863492;

        // Local constants
        const long double aa = AA_WGS84;
        const long double e2 = esquare_WGS84;
        const long double maxIter = 5;
        const long double convergenceFactor = 1.0e-9;
        const long double convergenceFactor_x_aa = convergenceFactor*aa;

        // Convert to lat lon alt
        const long double p = std::sqrt(X*X + Y*Y);
        long double lat = std::atan2(Z, p * (1.0 - e2));
        long double slat, N, altOld, latOld;

        long double alt = 0.0;

        for(int i = 0; i < maxIter; i++) {
            slat = std::sin(lat);
            N = aa / std::sqrt(1.0 - e2 * slat * slat);
            altOld = alt;
            alt = p/std::cos(lat) - N;
            latOld = lat;
            lat = std::atan2(Z, p * (1.0 - e2 * (N/(N+alt))));

            if( std::fabs(lat-latOld) < convergenceFactor && 
                std::fabs(alt-altOld) < convergenceFactor_x_aa) {
                    break;
            }
        }

        long double lon = std::atan2(Y,X);

        return Eigen::Vector3d(lat, lon, alt);
}

// Transform spherical to cartesian coordinates
Eigen::Vector3d sph2cart(
    const double az,
    const double elev,
    const double r
    ) {
        return Eigen::Vector3d (
            r * std::cos(elev) * std::cos(az),
            r * std::cos(elev) * std::sin(az),
            r * std::sin(elev)
        );
}

Eigen::Matrix3d DCM_ECEF2ENU(
    const Eigen::Vector3d& rIG          // Position of inertial origin with respect to ECEF origin expressed in ECEF frame
    ) {
        const Eigen::Vector3d latLonAlt = ECEFToLatLonAlt(rIG);
        const double lat = latLonAlt(0);
        const double lon = latLonAlt(1);
        const Eigen::Vector3d Robs = sph2cart(lon, lat, 1.0);
        const Eigen::Vector3d v_vert = Robs;
        const Eigen::Vector3d v_east = Eigen::Vector3d(0,0,1).cross(Robs).normalized();
        const Eigen::Vector3d v_north = v_vert.cross(v_east).normalized();

        return (Eigen::Matrix3d() << 
                    v_east.transpose(),
                    v_north.transpose(),
                    v_vert.transpose()
               ).finished();

}

Eigen::Vector3d transformENUToECEF(
    const Eigen::Vector3d& rIG,         // Position of inertial origin with respect to ECEF origin expressed in ECEF frame
    const Eigen::Vector3d& rI           // Vector with respect to inertial origin expressed in inertial frame   
    ) {
        const Eigen::Matrix3d R = DCM_ECEF2ENU(rIG).transpose();
        return rIG + R * rI;
}


// https://en.wikipedia.org/wiki/Cross_product#Conversion_to_matrix_multiplication
Eigen::Matrix3d crossProductEquivalent(
    const Eigen::Vector3d& r
    ) {
        return (Eigen::Matrix3d() << 0, -r(2), r(1), r(2), 0, -r(0), -r(1), r(0), 0).finished();
}

// Generates a rotation matrix from Axis-Angle specification
// https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
Eigen::Matrix3d rotationMatrix(
    const Eigen::Vector3d& r,           // Unit vector about which to rotate
    const double phi                    // Angle of rotation
    ) {
        return std::cos(phi) * Eigen::Matrix3d::Identity() + std::sin(phi) * crossProductEquivalent(r) + (1 - std::cos(phi)) * (r * r.transpose());
}

// Composes a rotation matrix that describes the rotation of B away from A
// 3-2-1 euler rotation. Angles are B rotated away from A
// Initially, frames A and B are aligned (old frame is initally frame A)
// Rotate away from old frame around z axis by zRotation
// Rotate away from old frame around y axis by yRotation
// Rotate away from old frame around x axis by xRotation
Eigen::Matrix3d eulerToRotation(
    const double xRotation,
    const double yRotation,
    const double zRotation 
    ) {
        Eigen::Matrix3d Rx = rotationMatrix(Eigen::Vector3d(1,0,0), xRotation);
        Eigen::Matrix3d Ry = rotationMatrix(Eigen::Vector3d(0,1,0), yRotation);
        Eigen::Matrix3d Rz = rotationMatrix(Eigen::Vector3d(0,0,1), zRotation);
    
        return Rz*Ry*Rx;
}

// Composes a direction cosine matrix that transforms a vector expressed in frame A to the same vector expressed in frame B
// 3-2-1 euler rotation. Angles are B rotated away from A
// Initially, frames A and B are aligned (old frame is initally frame A)
// Rotate away from old frame around z axis by zRotation
// Rotate away from old frame around y axis by yRotation
// Rotate away from old frame around x axis by xRotation
Eigen::Matrix3d eulerToDCM(
    const double xRotation,
    const double yRotation,
    const double zRotation 
    ) {
        Eigen::Matrix3d Rx = rotationMatrix(Eigen::Vector3d(1,0,0), xRotation);
        Eigen::Matrix3d Ry = rotationMatrix(Eigen::Vector3d(0,1,0), yRotation);
        Eigen::Matrix3d Rz = rotationMatrix(Eigen::Vector3d(0,0,1), zRotation);
    
        // Must transpose. If frame B is rotated away from frame A by a rotation R,
        // to rotate a vector expressed in frame A to a vector expressed in
        // frame B, must multiply by inv(R) = R'
        return (Rz*Ry*Rx).transpose();
}

// 3-2-1 euler rotation
// Initially, body and inertial frame are aligned (old frame is initally inertial)
// Rotate body away from old frame around z axis by zRotation
// Rotate body away from old frame around y axis by yRotation
// Rotate body away from old frame around x axis by xRotation
Eigen::Vector3d rotateBodyToENU(
    const Eigen::Vector3d& rB,
    const double xRotation,
    const double yRotation,
    const double zRotation
    ) {
        // Transpose since the DCM will take a vector from ENU -> body, but we want body -> ENU
        return eulerToDCM(xRotation, yRotation, zRotation).transpose() * rB; 
}

// 3-2-1 euler rotation
// Initially, body and inertial frame are aligned (old frame is initally inertial)
// Rotate body away from old frame around z axis by zRotation
// Rotate body away from old frame around y axis by yRotation
// Rotate body away from old frame around x axis by xRotation
Eigen::Vector3d transformBodyToENU(
    const Eigen::Vector3d& rBI,         // Position of body origin with respect to inertial origin expressed in inertial frame
    const Eigen::Vector3d& rB,          // Vector from body origin in body frame
    const double xRotation,                  
    const double yRotation,                 
    const double zRotation                    
    ) {

    return rBI + rotateBodyToENU(rB, xRotation, yRotation, zRotation);
}

// xRotation, yRotation, zRotation are 3-2-1 rotation of body away from ENU (inertial)
Eigen::Vector3d transformBodyToECEF(
    const Eigen::Vector3d& rIG,         // Position of body origin with respect to ECEF origin in ECEF frame
    const Eigen::Vector3d& rBI,         // Position of body origin with respect to inertial in inertial frame 
    const Eigen::Vector3d& rB,          // Vector with respect to body origin in body frame
    const double xRotation,             // 3-2-1 Euler rotation x-axis
    const double yRotation,             // 3-2-1 Euler rotation y-axis
    const double zRotation              // 3-2-1 Euler rotation z-axis
    ) {

        // Convert body coordinate to ENU
        const Eigen::Vector3d rcI = transformBodyToENU(rBI, rB, xRotation, yRotation, zRotation);

        // Convert ENU to ECEF
        const Eigen::Vector3d rcG = transformENUToECEF(rIG, rcI);

        return rcG;
}

// xRotation, yRotation, zRotation are 3-2-1 rotation of body away from ENU
// Returns rotation away from ECEF
Eigen::Vector4d composeECEFQuat(
    const Eigen::Vector3d rIG,          // Position of the ENU origin with respect to ECEF origin expressed in ECEF frame
    const double xRotation,             // 3-2-1 Euler rotation x-axis
    const double yRotation,             // 3-2-1 Euler rotation y-axis
    const double zRotation              // 3-2-1 Euler rotation z-axis
    ) {
        // Transpose of DCM is rotation of ECEF away from ENU
        // R1 = DCM_ECEF2ENU(rIG).transpose();
    
        // So the transpose of that is the rotation of ENU away from ECEF
        // R = DCM_ECEF2ENU(rIG);
        const Eigen::Quaterniond q1(DCM_ECEF2ENU(rIG));
        const Eigen::Quaterniond q2(eulerToDCM(xRotation, yRotation, zRotation));
        const Eigen::Quaterniond q = (q2 * q1).inverse().normalized();
        const Eigen::Matrix3d R1 = q.toRotationMatrix();

        const Eigen::Vector3d x = transformBodyToECEF(rIG, Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,0,0), xRotation, yRotation, zRotation) - rIG;
        const Eigen::Vector3d y = transformBodyToECEF(rIG, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,1,0), xRotation, yRotation, zRotation) - rIG;
        const Eigen::Vector3d z = transformBodyToECEF(rIG, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1), xRotation, yRotation, zRotation) - rIG;
        const Eigen::Matrix3d R2 = (Eigen::Matrix3d() << x, y, z).finished();

        std::cout << "R1" << std::endl;
        std::cout << R1 << std::endl;
        std::cout << "R2" << std::endl;
        std::cout << R2 << std::endl;

        return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}
