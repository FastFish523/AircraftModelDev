//
// Created by Administrator on 2026/2/2.
//

#include <cmath>
#include <iomanip>
#include <iostream>
#include "R11/R11Missile.h"


int main()
{
    constexpr double step = 0.005;
    constexpr double desiredImpactAngleDeg = 85.0;
    constexpr double impactAngleToleranceDeg = 0.05;
    constexpr double hitToleranceM = 0.01;
    constexpr double maxSimulationTime = 600.0;

    const Eigen::Vector3d targetLLA = {125.5, 0.0, 2.0};
    const Eigen::Vector3d missileLLA = {124.5, 0.0, 2.0};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);

    ModelDevelop::R11::Missile missile;
    missile.init(step, missileLLA);
    missile.setTargetEcf(targetPosEcf, {0.0, 0.0, 0.0});

    ModelDevelop::R11::TerminalImpactKinematicsConfig impactConfig;
    impactConfig.enable = true;
    impactConfig.startDistance = 60000.0;
    impactConfig.duration = 60.0;
    impactConfig.impactAngleDeg = desiredImpactAngleDeg;
    impactConfig.terminalSpeed = 1000.0;
    missile.setTerminalImpactKinematics(impactConfig);

    missile.launch(0.0, 0.0);

    const int maxSteps = static_cast<int>(maxSimulationTime / step);
    for (int i = 0; i < maxSteps; ++i) {
        const double terminalDistance = missile.update();
        if (terminalDistance >= 0.0) {
            const double signedFlightPathAngleDeg = missile.velocityTheta();
            const double impactAngleDeg = std::abs(signedFlightPathAngleDeg);
            const bool hit = terminalDistance <= hitToleranceM;
            const bool angleSatisfied = std::abs(impactAngleDeg - desiredImpactAngleDeg) <= impactAngleToleranceDeg;

            std::cout << std::fixed << std::setprecision(6)
                      << "terminal_dis:" << terminalDistance << '\n'
                      << "impact_flight_path_angle_deg:" << signedFlightPathAngleDeg << '\n'
                      << "impact_angle_deg:" << impactAngleDeg << '\n'
                      << "verification:" << (hit && angleSatisfied ? "PASS" : "FAIL") << std::endl;
            return hit && angleSatisfied ? 0 : 1;
        }
    }

    std::cerr << "verification:FAIL (simulation timeout)" << std::endl;
    return 2;
}
