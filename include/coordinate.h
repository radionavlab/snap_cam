#pragma once

#include <Eigen/Core>
#include <cmath>

/**
* This function was adapted from gss/shared/core/navtoolbox.
* X, Y, Z in meters in ECEF coordinate frame
* Lat, Lon in radians
* Alt in meters
*/
Eigen::Matrix<long double, 3, 1> ecef2LatLonAlt(const Eigen::Matrix<long double, 3, 1> pos) {

        const long double X = pos(0, 0);
        const long double Y = pos(1, 0);
        const long double Z = pos(2, 0);

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

        Eigen::Matrix<long double, 3, 1> LatLonAlt;
        LatLonAlt <<    lat,
                        lon,
                        alt;

        return LatLonAlt;
}



Eigen::Matrix<long double, 3, 3> R_body_to_ENU(
    const long double azimuth,
    const long double elevation) {

        Eigen::Matrix<long double, 3, 3> R_z;
        R_z <<  std::cos(azimuth),  -1*std::sin(azimuth),   0,
                std::sin(azimuth),  std::cos(azimuth),      0,
                0,                  0,                      1;

        Eigen::Matrix<long double, 3, 3> R_x;
        R_x <<  1,                  0,                      0,
                0,                  std::cos(elevation),    -1*std::sin(elevation),
                0,                  std::sin(elevation),    std::cos(elevation);

        return R_x * R_z;
}

Eigen::Matrix<long double, 3, 3> R_ENU_to_ECEF(
    const long double lat,
    const long double lon) {
        Eigen::Matrix<long double, 3, 3> R;
        R <<    -1*std::sin(lon),       -1*std::sin(lat) * std::cos(lon),       std::cos(lat) * std::cos(lon),
                std::cos(lon),          -1*std::sin(lat)*std::sin(lon),         std::cos(lat)*std::sin(lon),
                0,                      std::cos(lat),                          std::sin(lat);

        return R;

}

Eigen::Matrix<long double, 3, 1> camera_ECEF(
    const Eigen::Matrix<long double, 3, 1> primary_antenna_ECEF,
    const Eigen::Matrix<long double, 3, 1> camera_body,
    const long double azimuth,
    const long double elevation) {

        Eigen::Matrix<long double, 3, 1> LatLonAlt = ecef2LatLonAlt(primary_antenna_ECEF);
        const long double lat = LatLonAlt(0, 0);
        const long double lon = LatLonAlt(1, 0);
        const long double alt = LatLonAlt(2, 0);

        return  primary_antenna_ECEF + 
                R_ENU_to_ECEF(lat, lon) * R_body_to_ENU(azimuth, elevation) * camera_body;
}

