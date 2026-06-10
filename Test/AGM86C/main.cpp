//
// Created by Administrator on 2026/3/13.
//

#include <iostream>
#include "AGM86C/AGM86CMissile.h"


int main()
{
    const Eigen::Vector3d targetLLA    = {122.5,40,0};
    Eigen::Vector3d missileLLA            = {120,40,10*1000};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::AGM86C::Missile missile;
    missile.init(0.005,missileLLA);
    missile.setTargetEcf(targetPosEcf,{0,0,0},false);
    std::deque<Eigen::Vector3d> routePoints{};
    routePoints.push_back(Eigen::Vector3d(120.5,40.1,9000));
    routePoints.push_back(Eigen::Vector3d(121,40,7000));
    routePoints.push_back(Eigen::Vector3d(121.5,40,6000));
    routePoints.push_back(Eigen::Vector3d(121.9,40,4000));
    missile.setRoutePoints(routePoints);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    missile.launch(5,targetPsi);

    for(int i = 0;i<1000/0.005;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
