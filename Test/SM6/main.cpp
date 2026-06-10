//
// Created by Administrator on 2026/3/3.
//

#include <iostream>
#include "SM6/SM6Missile.h"


int main()
{
    const Eigen::Vector3d targetLLA    = {118,40,12000};
    Eigen::Vector3d missileLLA            = {120,40,2};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::SM6::Missile missile;
    missile.init(0.005,missileLLA);
    missile.setTargetEcf(targetPosEcf,{0,0,0},false);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    missile.launch(30,targetPsi);

    for(int i = 0;i<200/0.005;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
