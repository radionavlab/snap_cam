#include "GPS.h"

void attitudeMessageHandler(const mg_msgs::Attitude2D msg) {
    const double rx = msg.rx;
    const double ry = msg.ry;
    const double rz = msg.rz;
    const double rxRov = msg.rxRov;
    const double ryRov = msg.ryRov;
    const double rzRov = msg.rzRov;
    const mg_msgs::BaseTime tSolution = msg.tSolution;
    const double deltRSec = msg.deltRSec;
    const std::vector<float> P = msg.P;
    const uint32_t nCov = msg.nCov;
    const double azAngle = msg.azAngle;
    const double elAngle = msg.elAngle;
    const double azSigma = msg.azAngle;
    const double elSigma = msg.elSigma;
    const double testStat = msg.testStat;
    const uint8_t numDD = msg.numDD;
    const uint8_t bitfield = msg.bitfield;

    gpsSolution.el = elAngle;
    gpsSolution.az = azAngle;


    // Only have covariance information for pitch and yaw
    // Roll is assumed to be zero; 15 degree variance to account for uncertainty
    gpsSolution.attCov[0] = rollVar;
    gpsSolution.attCov[4] = elSigma * elSigma;
    gpsSolution.attCov[8] = azSigma * azSigma;
}

void positionMessageHandler(const mg_msgs::SingleBaselineRTK msg) {
    const double rx = msg.rx;
    const double ry = msg.ry;
    const double rz = msg.rz;
    const double rxRov = msg.rxRov;
    const double ryRov = msg.ryRov;
    const double rzRov = msg.rzRov;
    const mg_msgs::BaseTime tSolution = msg.tSolution;
    const double deltRSec = msg.deltRSec;
    const std::vector<float> P = msg.P;
    const uint32_t nCov = msg.nCov;
    const double testStat = msg.testStat;
    const double ageOfReferenceData = msg.ageOfReferenceData;
    const uint8_t numDD = msg.numDD;
    const uint8_t bitfield = msg.bitfield;

    gpsSolution.x = rxRov;
    gpsSolution.y = ryRov;
    gpsSolution.z = rzRov;

    // Copy the covariance
    for(int i = 0; i < 9; i++) {
        gpsSolution.posCov[i] = P[i];
    }
}
