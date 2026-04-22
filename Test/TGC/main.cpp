//
// Created by 17298 on 2026/4/22.
//

#include <iostream>
#include "TGC/TGCMissile.h"


int main() {
    double step                        = 0.005;
    const Eigen::Vector3d targetLLA    = {119.5, 40, 12000};
    Eigen::Vector3d missileLLA         = {120, 40, 2};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, missileLLA.x(), missileLLA.y());

    ModelDevelop::TGC::Missile missile;
    missile.init(step, missileLLA);
    missile.setTargetEcf(targetPosEcf, {0, 0, 0}, false);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue) * 57.3;
    missile.launch(30, targetPsi);

    for (int i = 0; i < 100 / step; i++) {
        double ret = missile.update();
        if (ret > 0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}