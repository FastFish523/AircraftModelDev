//
// Created by Administrator on 2026/3/12.
//

#include <iostream>
#include <cmath>
#include "BGM/BGMMissile.h"
#include "Seeker/Seeker.h"


int main()
{
    const Eigen::Vector3d targetLLA    = {122,40,0};
    Eigen::Vector3d missileLLA         = {121,40,2};
    constexpr auto cruiseAltitude      = 100.0;
    constexpr double cruiseMach = 0.7;
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::BGM::Missile missile;
    ModelDevelop::SEEKER::Seeker seeker;
    missile.init(0.005,missileLLA);
    missile.setCruiseAltAndMH(cruiseAltitude,cruiseMach);
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

    constexpr double pullBiasTriggerDistance = 20000.0; // m, distance to the target
    constexpr double pullBiasDuration = 15.0;           // s
    constexpr double pullBiasAngleDeg = 10.0;           // deg, right is positive

    const Eigen::Vector2d launchToTarget(targetPosNue.x(), targetPosNue.z());
    const Eigen::Vector2d forward = launchToTarget.normalized();
    const Eigen::Vector2d right(forward.y(), -forward.x());
    const Eigen::Vector2d firstHorizontal = launchToTarget - pullBiasTriggerDistance * forward;


    constexpr double speedOfSound = 340.0;
    constexpr double degToRad = 3.14159265358979323846 / 180.0;
    const double pullBiasDistance = cruiseMach * speedOfSound * pullBiasDuration;
    const double pullBiasAngleRad = pullBiasAngleDeg * degToRad;
    const Eigen::Vector2d pullBiasDirection =
        std::cos(pullBiasAngleRad) * forward + std::sin(pullBiasAngleRad) * right;
    const Eigen::Vector2d secondHorizontal = firstHorizontal + pullBiasDistance * pullBiasDirection;

    const auto toRoutePoint = [&](const Eigen::Vector2d &horizontal) {
        const Eigen::Vector3d pointNue(horizontal.x(), cruiseAltitude, horizontal.y());
        Eigen::Vector3d pointLla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(
            ModelDevelop::Utils::CoordinateHelper::nueToEcefPosition(
                pointNue, missileLLA.x(), missileLLA.y()));
        pointLla.z() = cruiseAltitude;
        return pointLla;
    };

    std::deque<Eigen::Vector3d> routePoints{
        toRoutePoint(firstHorizontal),
        toRoutePoint(secondHorizontal)
    };
    std::cout << "Route point 1 (lon, lat, alt): " << routePoints[0].transpose() << '\n'
              << "Route point 2 (lon, lat, alt): " << routePoints[1].transpose() << std::endl;
    missile.setRoutePoints(routePoints);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    missile.launch(45,targetPsi);

    for(int i = 0;i<1500/0.005;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
