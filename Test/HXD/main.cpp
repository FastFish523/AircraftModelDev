//
// Created by Administrator on 2026/1/29.
//

#include <iostream>
#include "HXD3530/Missile.h"


int main()
{
    double step=0.005;
    const Eigen::Vector3d targetLLA    = {110,24,35*1000};
    Eigen::Vector3d missileLLA            = {120,24,35*1000};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::HXD3530::Missile missile;
    missile.init(step,missileLLA);
    missile.setTargetEcf(targetPosEcf,{0,0,0});

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    missile.launch(0,targetPsi);

    for(int i = 0;i<1000/step;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
