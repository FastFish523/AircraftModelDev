//
// Created by Administrator on 2026/1/28.
//

#include <iostream>
#include "HACM/Missile.h"


int main()
{
    constexpr double CRUISE_VEL       = 2000.0;  // 目标巡航速度 [m/s],可以先写死
    // 如何基于最大转弯半径剔除不合理的路径点
    //桃园空军基地	纬度25.0556° 	经度121.2425°
    //正东80000m  	25.0556°	122.0325°	800m宜兰县以东约30公里的外海区域
    //正南370000m  	21.7281°	121.2425°	500m巴士海峡，菲律宾吕宋岛以北约350公里
    //西南200000m  	23.8050°	119.8535°	200m台湾海峡南部，澎湖列岛以北海域
    double step=0.005;
    const Eigen::Vector3d targetLLA    = {121,21,500};
    Eigen::Vector3d missileLLA            = {121,25,20000};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::HACM::Missile missile;
    missile.init(step,missileLLA);
    missile.setTargetEcf(targetPosEcf,{0,0,0});

    std::deque<Eigen::Vector3d> routes;
    routes.emplace_back(121,24.5,20000);
    routes.emplace_back(121.1,24,20000);
    routes.emplace_back(121.2,24,20000);
    routes.emplace_back(121.3,23,20000);
    routes.emplace_back(121.2,23,20000);
    routes.emplace_back(121.1,22,20000);
    missile.setRoutePoints(routes, CRUISE_VEL);


    const double targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;

    missile.launch(5,targetPsi, 1800, 20000, 300);

    for(int i = 0;i<1000/step;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
