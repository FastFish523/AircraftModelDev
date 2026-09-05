//
// Created by 17298 on 2026/4/22.
//

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include "HTV2/Engine.h"
#include "HTV2/HTV2Missile.h"
#include "../ScenarioArguments.h"
#include "Util/CoordinateHelper.h"

namespace {
    constexpr double kStep = 0.005;
    constexpr double kStatusPrintInterval = 200.0;
    constexpr double kGroundStopAltitude = -50.0;
    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

    struct Scenario {
        double launchLonDeg = 120.0;
        double launchLatDeg = 40.0;
        double launchAltitudeM = 10.0;
        double targetLonDeg = 140.0;
        double targetLatDeg = 20.0;
        double targetAltitudeM = 0.0;
        double launchThetaDeg = 89.0;
        double maxSimTimeS = 1000.0;
        std::string guidanceModule = "phase_pull_bias";
        std::string controlModule = "p6dof_pi";
        double guidanceNavigationConstant = 4.0;
        double guidanceHoldStartDistanceM = 80000.0;
        double guidanceHoldDurationS = 15.0;
        double guidanceMountAzDeg = 20.0;
        double guidanceMountElDeg = -10.0;
        double controlGainScale = 1.0;
        double controlRudderLimitDeg = 45.0;

        [[nodiscard]] Eigen::Vector3d launchLLA() const {
            return {launchLonDeg, launchLatDeg, launchAltitudeM};
        }

        [[nodiscard]] Eigen::Vector3d targetLLA() const {
            return {targetLonDeg, targetLatDeg, targetAltitudeM};
        }
    };

    ModelDevelop::TestSupport::ParseResult parseArguments(
        const int argc,
        char *argv[],
        Scenario &scenario) {
        return ModelDevelop::TestSupport::parseArguments(argc, argv, {
            {"--launch-lon-deg", &scenario.launchLonDeg, -180.0, 180.0, "deg", "Launch longitude"},
            {"--launch-lat-deg", &scenario.launchLatDeg, -89.9, 89.9, "deg", "Launch latitude"},
            {"--launch-altitude-m", &scenario.launchAltitudeM, 0.0, 100000.0, "m", "Launch altitude"},
            {"--target-lon-deg", &scenario.targetLonDeg, -180.0, 180.0, "deg", "Target longitude"},
            {"--target-lat-deg", &scenario.targetLatDeg, -89.9, 89.9, "deg", "Target latitude"},
            {"--target-altitude-m", &scenario.targetAltitudeM, -1000.0, 100000.0, "m", "Target altitude"},
            {"--launch-theta-deg", &scenario.launchThetaDeg, 1.0, 89.9, "deg", "Launch elevation angle"},
            {"--max-sim-time-s", &scenario.maxSimTimeS, 1.0, 2000.0, "s", "Maximum simulation time"},
            {"--guidance-navigation-constant", &scenario.guidanceNavigationConstant, 1.0, 8.0, "-", "Terminal proportional-navigation constant"},
            {"--guidance-hold-start-distance-m", &scenario.guidanceHoldStartDistanceM, 1000.0, 500000.0, "m", "Pull-bias hold start distance"},
            {"--guidance-hold-duration-s", &scenario.guidanceHoldDurationS, 0.1, 120.0, "s", "Pull-bias hold duration"},
            {"--guidance-mount-az-deg", &scenario.guidanceMountAzDeg, -180.0, 180.0, "deg", "Pull-bias sensor mount azimuth"},
            {"--guidance-mount-el-deg", &scenario.guidanceMountElDeg, -89.0, 89.0, "deg", "Pull-bias sensor mount elevation"},
            {"--control-gain-scale", &scenario.controlGainScale, 0.25, 2.0, "-", "P/PI feedback gain scale"},
            {"--control-rudder-limit-deg", &scenario.controlRudderLimitDeg, 5.0, 45.0, "deg", "Absolute rudder deflection limit"},
        }, {
            {"--guidance-module", &scenario.guidanceModule, {"phase_pull_bias", "phase_standard"}, "Guidance module"},
            {"--control-module", &scenario.controlModule, {"p6dof_pi", "p6dof_p"}, "Control module"},
        });
    }

    void printMissileState(const std::string &tag, const ModelDevelop::HTV2::Missile &missile) {
        const auto lla = missile.lla();
        const auto attitude = missile.attitudeEuler();

        std::cout << tag
                  << "phase=" << missile.phaseName()
                  << ", t=" << missile.flyTime()
                  << "s, V=" << missile.V()
                  << "m/s, alt=" << lla.z()
                  << "m, dis=" << missile.targetDis()
                  << "m, theta=" << missile.velocityTheta()
                  << "deg, psi=" << missile.velocityPsi()
                  << "deg, pitch=" << attitude.y()
                  << "deg, mass=" << missile.mass()
                  << "kg, P=" << missile.P()
                  << "N" << std::endl;
    }

    void printBoostState(const int stage, const ModelDevelop::HTV2::Missile &missile) {
        printMissileState("[stage " + std::to_string(stage) + " burnout] ", missile);
    }

    double calculateLaunchPsi(const Eigen::Vector3d &missileLLA, const Eigen::Vector3d &targetLLA) {
        const auto targetEcef = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
        const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(
            targetEcef, missileLLA.x(), missileLLA.y());
        return ModelDevelop::Utils::CoordinateHelper::getPsi(relNue) * kRadToDeg;
    }

    bool hasValidHorizontalSeparation(
        const Eigen::Vector3d &missileLLA,
        const Eigen::Vector3d &targetLLA) {
        const auto targetEcef = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
        const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(
            targetEcef, missileLLA.x(), missileLLA.y());
        const double horizontalDistance = std::hypot(relNue.x(), relNue.z());
        if (!std::isfinite(horizontalDistance) || horizontalDistance < 1.0) {
            std::cerr << "Launch and target horizontal separation must be at least 1 m."
                      << std::endl;
            return false;
        }
        return true;
    }

    ModelDevelop::HTV2::Missile createMissile(
        const double step,
        const double launchPsi,
        const Scenario &scenario) {
        ModelDevelop::HTV2::Missile missile;
        missile.init(step, scenario.launchLLA());
        missile.setTargetLLA(scenario.targetLLA(), Eigen::Vector3d::Zero(), true);
        missile.launch(scenario.launchThetaDeg, launchPsi);
        return missile;
    }

    void printSimulationHeader(const Scenario &scenario, const double launchPsi) {
        const auto burnOutTimes = ModelDevelop::HTV2::Engine::boostBurnOutTimes();

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "HTV2 full trajectory test started" << std::endl;
        std::cout << "launch lla: [" << scenario.launchLonDeg << ", " << scenario.launchLatDeg << ", " << scenario.launchAltitudeM << "]" << std::endl;
        std::cout << "target lla: [" << scenario.targetLonDeg << ", " << scenario.targetLatDeg << ", " << scenario.targetAltitudeM << "]" << std::endl;
        std::cout << "launchTheta: " << scenario.launchThetaDeg << " deg" << std::endl;
        std::cout << "launchPsi: " << launchPsi << " deg" << std::endl;
        std::cout << "maxSimTime: " << scenario.maxSimTimeS << " s" << std::endl;
        std::cout << "guidanceModule: " << scenario.guidanceModule << std::endl;
        std::cout << "guidanceNavigationConstant: " << scenario.guidanceNavigationConstant << std::endl;
        std::cout << "guidanceHoldStartDistance: " << scenario.guidanceHoldStartDistanceM << " m" << std::endl;
        std::cout << "guidanceHoldDuration: " << scenario.guidanceHoldDurationS << " s" << std::endl;
        std::cout << "guidanceMountAz: " << scenario.guidanceMountAzDeg << " deg" << std::endl;
        std::cout << "guidanceMountEl: " << scenario.guidanceMountElDeg << " deg" << std::endl;
        std::cout << "controlModule: " << scenario.controlModule << std::endl;
        std::cout << "controlGainScale: " << scenario.controlGainScale << std::endl;
        std::cout << "controlRudderLimit: " << scenario.controlRudderLimitDeg << " deg" << std::endl;
        std::cout << "boost burnout schedule(s): [" << burnOutTimes[0] << ", " << burnOutTimes[1] << ", " << burnOutTimes[2] << "]" << std::endl;
    }

    void printPhaseChange(const ModelDevelop::HTV2::Missile &missile) {
        printMissileState("[phase change] ", missile);
    }

    void printFinalSummary(
        const ModelDevelop::HTV2::Missile &missile,
        const double finalMissDistance,
        const double closestMissDistance) {
        printMissileState("[final] ", missile);
        std::cout << "[miss] final distance=" << finalMissDistance
                  << "m, closest distance=" << closestMissDistance
                  << "m" << std::endl;
    }
}

int main(const int argc, char *argv[]) {
    using ModelDevelop::HTV2::Engine;

    Scenario scenario;
    const auto parseResult = parseArguments(argc, argv, scenario);
    if (parseResult == ModelDevelop::TestSupport::ParseResult::Help) {
        return 0;
    }
    if (parseResult == ModelDevelop::TestSupport::ParseResult::Error) {
        return 2;
    }

    if (!hasValidHorizontalSeparation(scenario.launchLLA(), scenario.targetLLA())) {
        return 2;
    }

    const double launchPsi = calculateLaunchPsi(scenario.launchLLA(), scenario.targetLLA());
    auto missile = createMissile(kStep, launchPsi, scenario);
    const auto burnOutTimes = Engine::boostBurnOutTimes();
    ModelDevelop::HTV2::TerminalAttitudeHoldConfig holdConfig;
    holdConfig.enable = scenario.guidanceModule == "phase_pull_bias";
    holdConfig.startDistance = scenario.guidanceHoldStartDistanceM;//拉偏触发距离
    holdConfig.duration = scenario.guidanceHoldDurationS;//拉偏持续时间
    holdConfig.holdInitialAttitude = true;
    holdConfig.view.pointBody = {0.0, 0.0, 0.0};
    holdConfig.view.useMountAngles = true;
    holdConfig.view.mountAzDeg = scenario.guidanceMountAzDeg;//安装角（偏航）
    holdConfig.view.mountElDeg = scenario.guidanceMountElDeg;//安装角（俯仰）
    holdConfig.view.coneAngleDeg = 5.0;//视场对准阈值
    // holdConfig.view.reacquireConeAngleDeg = 3.0;//重新捕获阈值
    // holdConfig.view.releaseInsideCone = false;
    // holdConfig.ki = {0.0, 0.0, 0.0};
    // holdConfig.integralLimit = {0.2, 0.2, 0.2};
    ModelDevelop::HTV2::GuidanceModuleConfig guidanceConfig;
    guidanceConfig.module = scenario.guidanceModule == "phase_pull_bias"
        ? ModelDevelop::HTV2::GuidanceModule::PhasePullBias
        : ModelDevelop::HTV2::GuidanceModule::PhaseStandard;
    guidanceConfig.terminalPnNavigationConstant = scenario.guidanceNavigationConstant;
    missile.configureGuidance(guidanceConfig, holdConfig);

    ModelDevelop::HTV2::ControlModuleConfig controlConfig;
    controlConfig.module = scenario.controlModule == "p6dof_pi"
        ? ModelDevelop::HTV2::ControlModule::P6dofPi
        : ModelDevelop::HTV2::ControlModule::P6dofP;
    controlConfig.gainScale = scenario.controlGainScale;
    controlConfig.rudderLimitDeg = scenario.controlRudderLimitDeg;
    missile.configureControl(controlConfig);

    printSimulationHeader(scenario, launchPsi);
    printPhaseChange(missile);

    double nextStatusPrintTime = kStatusPrintInterval;
    size_t nextBurnOut = 0;
    std::string lastPhase = missile.phaseName();
    double closestMissDistance = missile.targetDis();
    double finalMissDistance = closestMissDistance;

    while (missile.flyTime() < scenario.maxSimTimeS - kStep * 0.5) {
        const double terminalMissDistance = missile.update();
        const double currentDistance = missile.targetDis();
        closestMissDistance = std::min(closestMissDistance, currentDistance);
        finalMissDistance = terminalMissDistance >= 0.0 ? terminalMissDistance : currentDistance;

        const std::string phase = missile.phaseName();
        if (phase != lastPhase) {
            printPhaseChange(missile);
            lastPhase = phase;
        }

        while (missile.flyTime() + kStep * 0.5 >= nextStatusPrintTime) {
            printMissileState("[status] ", missile);
            nextStatusPrintTime += kStatusPrintInterval;
        }

        while (nextBurnOut < burnOutTimes.size() && missile.flyTime() + kStep * 0.5 >= burnOutTimes[nextBurnOut]) {
            printBoostState(static_cast<int>(nextBurnOut + 1), missile);
            ++nextBurnOut;
        }

        if (missile.lla().z() < kGroundStopAltitude) {
            std::cout << "simulation stopped because missile altitude dropped below ground." << std::endl;
            break;
        }
        if (terminalMissDistance >= 0.0) {
            std::cout << "simulation stopped after closest approach." << std::endl;
            break;
        }
    }

    printFinalSummary(missile, finalMissDistance, closestMissDistance);
    return 0;
}
