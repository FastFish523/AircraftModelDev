//
// Created by Administrator on 2026/3/12.
//

#include <iostream>
#include <cmath>
#include <iomanip>
#include <string>
#include "BGM/BGMMissile.h"
#include "../ScenarioArguments.h"
#include "Seeker/Seeker.h"

namespace {
    constexpr double kStep = 0.005;

    struct Scenario {
        double launchLonDeg = 121.0;
        double launchLatDeg = 40.0;
        double launchAltitudeM = 2.0;
        double targetLonDeg = 122.0;
        double targetLatDeg = 40.0;
        double targetAltitudeM = 0.0;
        double launchThetaDeg = 45.0;
        double maxSimTimeS = 1500.0;
        double cruiseAltitudeM = 100.0;
        double cruiseMach = 0.7;
        std::string guidanceModule = "phase_l1";
        std::string controlModule = "p6dof_pi";
        double guidanceNavigationConstant = 4.0;
        double guidanceL1LookaheadFactor = 5.0;
        double guidanceFirstWaypointDistanceM = 20000.0;
        double guidancePullBiasAngleDeg = 10.0;
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
            {"--cruise-altitude-m", &scenario.cruiseAltitudeM, 10.0, 20000.0, "m", "Cruise altitude"},
            {"--cruise-mach", &scenario.cruiseMach, 0.1, 5.0, "Mach", "Cruise Mach number"},
            {"--guidance-navigation-constant", &scenario.guidanceNavigationConstant, 1.0, 8.0, "-", "Terminal/fallback proportional-navigation constant"},
            {"--guidance-l1-lookahead-factor", &scenario.guidanceL1LookaheadFactor, 1.0, 20.0, "-", "L1 lookahead multiplier on minimum turn radius"},
            {"--guidance-first-waypoint-distance-m", &scenario.guidanceFirstWaypointDistanceM, 1000.0, 100000.0, "m", "First pull-bias waypoint distance to target"},
            {"--guidance-pull-bias-angle-deg", &scenario.guidancePullBiasAngleDeg, -60.0, 60.0, "deg", "Pull-bias route angle; right is positive"},
            {"--control-gain-scale", &scenario.controlGainScale, 0.25, 2.0, "-", "P/PI feedback gain scale"},
            {"--control-rudder-limit-deg", &scenario.controlRudderLimitDeg, 5.0, 45.0, "deg", "Absolute rudder deflection limit"},
        }, {
            {"--guidance-module", &scenario.guidanceModule, {"phase_l1", "phase_pn"}, "Guidance module"},
            {"--control-module", &scenario.controlModule, {"p6dof_pi", "p6dof_p"}, "Control module"},
        });
    }

    void printScenario(const Scenario &scenario, const double launchPsiDeg) {
        std::cout << std::fixed << std::setprecision(2)
                  << "BGM trajectory test started\n"
                  << "launch lla: [" << scenario.launchLonDeg << ", " << scenario.launchLatDeg << ", " << scenario.launchAltitudeM << "]\n"
                  << "target lla: [" << scenario.targetLonDeg << ", " << scenario.targetLatDeg << ", " << scenario.targetAltitudeM << "]\n"
                  << "launchTheta: " << scenario.launchThetaDeg << " deg\n"
                  << "launchPsi: " << launchPsiDeg << " deg\n"
                  << "maxSimTime: " << scenario.maxSimTimeS << " s\n"
                  << "cruiseAltitude: " << scenario.cruiseAltitudeM << " m\n"
                  << "cruiseMach: " << scenario.cruiseMach << "\n"
                  << "guidanceModule: " << scenario.guidanceModule << "\n"
                  << "guidanceNavigationConstant: " << scenario.guidanceNavigationConstant << "\n"
                  << "guidanceL1LookaheadFactor: " << scenario.guidanceL1LookaheadFactor << "\n"
                  << "guidanceFirstWaypointDistance: " << scenario.guidanceFirstWaypointDistanceM << " m\n"
                  << "guidancePullBiasAngle: " << scenario.guidancePullBiasAngleDeg << " deg\n"
                  << "controlModule: " << scenario.controlModule << "\n"
                  << "controlGainScale: " << scenario.controlGainScale << "\n"
                  << "controlRudderLimit: " << scenario.controlRudderLimitDeg << " deg" << std::endl;
    }
}

int main(const int argc, char *argv[])
{
    Scenario scenario;
    const auto parseResult = parseArguments(argc, argv, scenario);
    if (parseResult == ModelDevelop::TestSupport::ParseResult::Help) {
        return 0;
    }
    if (parseResult == ModelDevelop::TestSupport::ParseResult::Error) {
        return 2;
    }

    const Eigen::Vector3d targetLLA = scenario.targetLLA();
    const Eigen::Vector3d missileLLA = scenario.launchLLA();
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());
    const double horizontalDistance = std::hypot(targetPosNue.x(), targetPosNue.z());
    if (!std::isfinite(horizontalDistance) || horizontalDistance < 1.0) {
        std::cerr << "Launch and target horizontal separation must be at least 1 m."
                  << std::endl;
        return 2;
    }

    ModelDevelop::BGM::Missile missile;
    ModelDevelop::SEEKER::Seeker seeker;
    missile.init(kStep,missileLLA);
    ModelDevelop::BGM::GuidanceModuleConfig guidanceConfig;
    guidanceConfig.module = scenario.guidanceModule == "phase_l1"
        ? ModelDevelop::BGM::GuidanceModule::PhaseL1
        : ModelDevelop::BGM::GuidanceModule::PhasePn;
    guidanceConfig.pnNavigationConstant = scenario.guidanceNavigationConstant;
    guidanceConfig.l1LookaheadFactor = scenario.guidanceL1LookaheadFactor;
    missile.configureGuidance(guidanceConfig);

    ModelDevelop::BGM::ControlModuleConfig controlConfig;
    controlConfig.module = scenario.controlModule == "p6dof_pi"
        ? ModelDevelop::BGM::ControlModule::P6dofPi
        : ModelDevelop::BGM::ControlModule::P6dofP;
    controlConfig.gainScale = scenario.controlGainScale;
    controlConfig.rudderLimitDeg = scenario.controlRudderLimitDeg;
    missile.configureControl(controlConfig);

    missile.setCruiseAltAndMH(scenario.cruiseAltitudeM, scenario.cruiseMach);
    missile.setGetLOSInfoFunction( [&](
        const Eigen::Vector3d& _targetPosEcf,
        const Eigen::Vector3d& _targetVelEcf,
        const State& _state
    ) -> ModelDevelop::BGM::LosInfo {
        ModelDevelop::SEEKER::LosInfo tmp=seeker.getLOSInfo(_targetPosEcf,_targetVelEcf,_state);
        ModelDevelop::BGM::LosInfo re={
            tmp.sigma_elv,
            tmp.sigma_az,
            tmp.sigma_elv_dot,
            tmp.sigma_az_dot,
            tmp.dis_dot,
            tmp.sigma_elv_b,
            tmp.sigma_az_b
        };
        return re;
    });
    missile.setTargetEcf(targetPosEcf,{0,0,0},false);

    if (guidanceConfig.module == ModelDevelop::BGM::GuidanceModule::PhaseL1) {
        const Eigen::Vector2d launchToTarget(targetPosNue.x(), targetPosNue.z());
        const Eigen::Vector2d forward = launchToTarget.normalized();
        const Eigen::Vector2d right(forward.y(), -forward.x());
        const Eigen::Vector2d firstHorizontal = launchToTarget - scenario.guidanceFirstWaypointDistanceM * forward;

        constexpr double speedOfSound = 340.0;
        constexpr double degToRad = 3.14159265358979323846 / 180.0;
        const double pullBiasDistance = scenario.cruiseMach * speedOfSound * ModelDevelop::BGM::GuidanceTiming::PULL_BIAS_DURATION_S;
        const double pullBiasAngleRad = scenario.guidancePullBiasAngleDeg * degToRad;
        const Eigen::Vector2d pullBiasDirection =
            std::cos(pullBiasAngleRad) * forward + std::sin(pullBiasAngleRad) * right;
        const Eigen::Vector2d secondHorizontal = firstHorizontal + pullBiasDistance * pullBiasDirection;

        const auto toRoutePoint = [&](const Eigen::Vector2d &horizontal) {
            const Eigen::Vector3d pointNue(horizontal.x(), scenario.cruiseAltitudeM, horizontal.y());
            Eigen::Vector3d pointLla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(
                ModelDevelop::Utils::CoordinateHelper::nueToEcefPosition(
                    pointNue, missileLLA.x(), missileLLA.y()));
            pointLla.z() = scenario.cruiseAltitudeM;
            return pointLla;
        };

        std::deque<Eigen::Vector3d> routePoints{
            toRoutePoint(firstHorizontal),
            toRoutePoint(secondHorizontal)
        };
        std::cout << "Route point 1 (lon, lat, alt): " << routePoints[0].transpose() << '\n'
                  << "Route point 2 (lon, lat, alt): " << routePoints[1].transpose() << std::endl;
        missile.setRoutePoints(routePoints);
    }

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    printScenario(scenario, targetPsi);
    missile.launch(scenario.launchThetaDeg, targetPsi);

    while (missile.flyTime() < scenario.maxSimTimeS - kStep * 0.5) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
