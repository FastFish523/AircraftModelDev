//
// Created by Administrator on 2026/3/12.
//

#include <iostream>
#include "BGM/BGMMissile.h"
#include "Seeker/Seeker.h"


int main()
{
    const Eigen::Vector3d targetLLA    = {122,40,0};
    Eigen::Vector3d missileLLA            = {121,40,2};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::BGM::Missile missile;
    ModelDevelop::SEEKER::Seeker seeker;
    missile.init(0.005,missileLLA);
    missile.setCruiseAltAndMH(250,0.7);
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

    std::deque<Eigen::Vector3d> routePoints{};
    routePoints.push_back(Eigen::Vector3d(121.2,40.2,200));
    routePoints.push_back(Eigen::Vector3d(121.5,40.1,200));
    routePoints.push_back(Eigen::Vector3d(121.98,40,200));
    missile.setRoutePoints(routePoints);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    missile.launch(89,targetPsi);

    for(int i = 0;i<1500/0.005;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
