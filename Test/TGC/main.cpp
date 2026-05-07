//
// Created by 17298 on 2026/4/22.
//

#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>

#include "TGC/TGCMissile.h"

int main() {
    using ModelDevelop::TGC::GuidancePhase;
    using ModelDevelop::TGC::Missile;
    using ModelDevelop::Utils::CoordinateHelper;

    constexpr double step = 0.01;
    constexpr double maxSimTime = 900.0;

    const Eigen::Vector3d missileLLA = {120.0, 40.0, 10.0};
    //const Eigen::Vector3d targetLLA = {119.20, 40.60, 0.0};
    const Eigen::Vector3d targetLLA = {119.50, 40.40, 100.0};
    const Eigen::Vector3d targetVelEcf = Eigen::Vector3d::Zero();
    const Eigen::Vector3d targetPosEcf = CoordinateHelper::llaToEcef(targetLLA);

    std::deque<Eigen::Vector3d> routePoints = {
        {119.85, 40.12, 25000.0},
        {119.55, 40.32, 11000.0}
        //{119.35, 40.48, 8000.0},
        //{119.24, 40.56, 4000.0}
    };

    const Eigen::Vector3d targetPosNue = CoordinateHelper::ecefToNuePosition(targetPosEcf, missileLLA.x(), missileLLA.y());
    const double launchPsi = CoordinateHelper::getPsi(targetPosNue) * 57.3;

    Missile missile;
    missile.init(step, missileLLA);
    missile.setTargetEcf(targetPosEcf, targetVelEcf, true);
    missile.setRoutePoints(routePoints);
    missile.launch(25.0, launchPsi);

    GuidancePhase lastPhase = missile.phase();
    double minDistance = std::numeric_limits<double>::max();
    double nextReportTime = 0.0;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "TGC full mission test started" << std::endl;
    std::cout << "launch lla: [" << missileLLA.x() << ", " << missileLLA.y() << ", " << missileLLA.z() << "]" << std::endl;
    std::cout << "target lla: [" << targetLLA.x() << ", " << targetLLA.y() << ", " << targetLLA.z() << "]" << std::endl;
    std::cout << "initial heading psi(deg): " << launchPsi << std::endl;

    for (int i = 0; i < static_cast<int>(maxSimTime / step); ++i) {
        const double terminalDistance = missile.update();
        minDistance = std::min(minDistance, missile.targetDis());

        if (missile.phase() != lastPhase) {
            lastPhase = missile.phase();
            std::cout << "[phase] t=" << missile.flyTime()
                      << "s -> " << missile.phaseName()
                      << ", range=" << missile.targetDis()
                      << "m, alt=" << missile.lla().z()
                      << "m" << std::endl;
        }

        if (missile.flyTime() >= nextReportTime) {
            std::cout << "[status] t=" << missile.flyTime()
                      << "s, phase=" << missile.phaseName()
                      << ", range=" << missile.targetDis()
                      << "m, V=" << missile.V()
                      << "m/s, alt=" << missile.lla().z()
                      << "m, theta=" << missile.velocityTheta()
                      << "deg, psi=" << missile.velocityPsi()
                      << "deg" << std::endl;
            nextReportTime += 5.0;
        }

        if (terminalDistance > 0.0) {
            std::cout << "terminal distance: " << terminalDistance << " m" << std::endl;
            std::cout << "minimum distance: " << minDistance << " m" << std::endl;
            std::cout << "flight time: " << missile.flyTime() << " s" << std::endl;
            return 0;
        }

        if (missile.lla().z() < -50.0) {
            std::cout << "simulation stopped because missile altitude dropped below ground." << std::endl;
            break;
        }
    }

    std::cout << "simulation finished without terminal hit." << std::endl;
    std::cout << "final phase: " << missile.phaseName() << std::endl;
    std::cout << "final range: " << missile.targetDis() << " m" << std::endl;
    std::cout << "minimum distance: " << minDistance << " m" << std::endl;
    std::cout << "final altitude: " << missile.lla().z() << " m" << std::endl;
    return 0;
}
