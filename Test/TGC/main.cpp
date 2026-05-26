//
// Full trajectory validation runner.
//

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "TGC/BoostConfig.h"
#include "TGC/Engine.h"
#include "TGC/TGCMissile.h"
#include "Util/CoordinateHelper.h"

namespace {
    double calcPsiToTarget(const Eigen::Vector3d &missileLLA, const Eigen::Vector3d &targetLLA) {
        const auto targetEcef = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
        const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(
            targetEcef,
            missileLLA.x(),
            missileLLA.y());
        return ModelDevelop::Utils::CoordinateHelper::getPsi(relNue) * 180.0 / std::acos(-1.0);
    }

    void printBoostState(const int stage, const ModelDevelop::TGC::Missile &missile) {
        const auto lla = missile.lla();
        const auto attitude = missile.attitudeEuler();
        const double theta = missile.velocityTheta();

        std::cout << "[stage " << stage << " burnout] "
                  << "t=" << missile.flyTime()
                  << "s, phase=" << missile.phaseName()
                  << ", V=" << missile.V()
                  << "m/s, alt=" << lla.z()
                  << "m, theta=" << theta
                  << "deg, vertical_offset=" << std::abs(90.0 - theta)
                  << "deg, pitch=" << attitude.y()
                  << "deg, mass=" << missile.mass()
                  << "kg, P=" << missile.P()
                  << "N" << std::endl;
    }

    void printMissionState(const ModelDevelop::TGC::Missile &missile) {
        const auto lla = missile.lla();
        std::cout << "[mission] "
                  << "t=" << missile.flyTime()
                  << "s, phase=" << missile.phaseName()
                  << ", V=" << missile.V()
                  << "m/s, alt=" << lla.z()
                  << "m, theta=" << missile.velocityTheta()
                  << "deg, thetaCmd=" << missile.thetaCmd()
                  << "deg, psi=" << missile.velocityPsi()
                  << "deg, targetDis=" << missile.targetDis()
                  << "m, accCmdY=" << missile.acc_cmd_b_y()
                  << "m/s2, accCmdZ=" << missile.acc_cmd_b_z()
                  << "m/s2, sigmaAz=" << missile.sigmaAz()
                  << "deg, sigmaElv=" << missile.sigmaElv()
                  << "deg" << std::endl;
    }
}

int main() {
    using ModelDevelop::TGC::Engine;
    using ModelDevelop::TGC::Missile;

    constexpr double step = 0.02;
    constexpr double maxSimTime = 1200.0;
    constexpr double reportInterval = 500.0;

    const Eigen::Vector3d missileLLA = {120.0, 40.0, 10.0};
    const Eigen::Vector3d targetLLA = {160.0, 5.0, 0.0};
    const auto burnOutTimes = Engine::boostBurnOutTimes();
    const double launchPsi = calcPsiToTarget(missileLLA, targetLLA);

    Missile missile;
    missile.init(step, missileLLA);
    missile.setTargetLLA(targetLLA, Eigen::Vector3d::Zero(), true);
    missile.launch(ModelDevelop::TGC::BoostConfig::LAUNCH_THETA_F, launchPsi);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "TGC full trajectory simulation started" << std::endl;
    std::cout << "launch lla: [" << missileLLA.x() << ", " << missileLLA.y() << ", " << missileLLA.z() << "]" << std::endl;
    std::cout << "target lla: [" << targetLLA.x() << ", " << targetLLA.y() << ", " << targetLLA.z() << "]" << std::endl;
    std::cout << "launchPsi: " << launchPsi << " deg" << std::endl;
    std::cout << "boost burnout schedule(s): [" << burnOutTimes[0] << ", " << burnOutTimes[1] << ", " << burnOutTimes[2] << "]" << std::endl;

    size_t nextBurnOut = 0;
    double nextReportTime = 0.0;
    std::string lastPhase = missile.phaseName();

    while (missile.flyTime() < maxSimTime - step * 0.5) {
        const double terminalDis = missile.update();

        while (nextBurnOut < burnOutTimes.size() && missile.flyTime() + step * 0.5 >= burnOutTimes[nextBurnOut]) {
            printBoostState(static_cast<int>(nextBurnOut + 1), missile);
            ++nextBurnOut;
        }

        const std::string phase = missile.phaseName();
        if (phase != lastPhase) {
            std::cout << "[phase change] t=" << missile.flyTime()
                      << "s, phase=" << missile.phaseName() << std::endl;
            printMissionState(missile);
            lastPhase = phase;
        }

        if (missile.flyTime() + step * 0.5 >= nextReportTime) {
            printMissionState(missile);
            nextReportTime += reportInterval;
        }

        if (terminalDis >= 0.0) {
            std::cout << "simulation stopped by terminal distance, miss distance=" << terminalDis << "m" << std::endl;
            break;
        }

        if (missile.lla().z() < -50.0) {
            std::cout << "simulation stopped because missile altitude dropped below ground." << std::endl;
            break;
        }
    }

    std::cout << "[final] t=" << missile.flyTime()
              << "s, phase=" << missile.phaseName()
              << ", V=" << missile.V()
              << "m/s, alt=" << missile.lla().z()
              << "m, theta=" << missile.velocityTheta()
              << "deg, targetDis=" << missile.targetDis()
              << "m, mass=" << missile.mass()
              << "kg, P=" << missile.P()
              << "N" << std::endl;
    return 0;
}
