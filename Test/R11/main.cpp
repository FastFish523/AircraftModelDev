//
// Created by Administrator on 2026/2/2.
//

#include <iostream>
#include "R11/R11Missile.h"


int main()
{
    const Eigen::Vector3d targetLLA    = {125.5,0,2};
    Eigen::Vector3d missileLLA            = {124.5,0,2};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::R11::Missile missile;
    missile.init(0.005,missileLLA);
    missile.setTargetEcf(targetPosEcf,{0,0,0});
    ModelDevelop::R11::TerminalAttitudeHoldConfig holdConfig;
    holdConfig.enable = true;
    holdConfig.startDistance = 60000.0;//拉偏触发距离
    holdConfig.duration = 15.0;//拉偏持续时间
    holdConfig.view.pointBody = {0.0, 0.0, 0.0};
    holdConfig.view.useMountAngles = true;
    holdConfig.view.mountAzDeg = 10.0;//安装角（偏航）
    holdConfig.view.mountElDeg = -10.0;//安装角（俯仰）
    holdConfig.view.coneAngleDeg = 5.0;//视场对准阈值
    // holdConfig.view.reacquireConeAngleDeg = 3.0;//重新捕获阈值
    // holdConfig.view.releaseInsideCone = false;
    // holdConfig.ki = {0.0, 0.0, 0.0};
    // holdConfig.integralLimit = {0.2, 0.2, 0.2};
    missile.setTerminalAttitudeHold(holdConfig);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    missile.launch(0,0);

    for(int i = 0;i<15000/0.005;i++) {
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
