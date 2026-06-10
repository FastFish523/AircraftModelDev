//
// Created by Administrator on 2026/4/9.
//

#include <iostream>
#include "TheoreticalModel/TheoreticalModelMissile.h"


int main()
{
    double step = 0.005;
    const Eigen::Vector3d targetLLA    = {119.5,40,3000};
    Eigen::Vector3d missileLLA            = {120,40,3000};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::TheoreticalModel::Missile missile;
    missile.init(step,missileLLA);
    missile.setTargetEcf(targetPosEcf,{0,0,0},false);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    // const auto targetTheta = ModelDevelop::Utils::CoordinateHelper::getTheta(targetPosNue)*57.3;
    missile.launch(0,targetPsi);

    for(int i = 0;i<100/step;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
