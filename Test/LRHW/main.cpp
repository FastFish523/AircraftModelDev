//
// Created by Administrator on 2026/3/16.
//

#include <iostream>
#include "LRHW/LRHWMissile.h"


int main()
{
    double step = 0.005;
    const Eigen::Vector3d targetLLA    = {120.8,55.5,2};
    Eigen::Vector3d missileLLA            = {120,40,2};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::LRHW::Missile missile;
    missile.init(step,missileLLA);
    missile.setTargetEcf(targetPosEcf,{0,0,0},false);

    std::deque<Eigen::Vector3d> routePoints{};
    routePoints.push_back(Eigen::Vector3d(120.2,45,40000));
    routePoints.push_back(Eigen::Vector3d(120.4,50,30000));
    routePoints.push_back(Eigen::Vector3d(120.6,55,20000));
    missile.setRoutePoints(routePoints);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    missile.launch(35,targetPsi);
    std::cout << "start dis:" << missile.targetDis()/1000.0<<" km" << std::endl;
    for(int i = 0;i<1200/step;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;

}
