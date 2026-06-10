//
// Created by Administrator on 2026/3/11.
//

#include <iostream>
#include <HXD3530/Missile.h>

#include "PAC500/Missile.h"


int main()
{
    double step = 0.005;
    Eigen::Vector3d missileLLA            = {120,24,2};

    const Eigen::Vector3d hxdLLA    = {121,26,35*1000};
    const Eigen::Vector3d hxdTargetLLA    = {100,26,35*1000};
    const Eigen::Vector3d hxdTargetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(hxdTargetLLA);
    const Eigen::Vector3d hxdTargetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(hxdTargetPosEcf,hxdLLA.x(),hxdLLA.y());
    const Eigen::Vector3d hxdPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(hxdLLA);
    const Eigen::Vector3d hxdPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(hxdPosEcf,missileLLA.x(),missileLLA.y());

    ModelDevelop::HXD3530::Missile hxd;
    hxd.init(step,hxdLLA);
    hxd.setTargetEcf(hxdTargetPosEcf,{0,0,0});
    const auto hxdTargetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(hxdTargetPosNue)*57.3;
    hxd.launch(0,hxdTargetPsi);


    ModelDevelop::PAC500::Missile missile;
    missile.init(0.005,missileLLA);
    missile.setTargetEcf(hxdPosEcf,{0,0,0},false);

    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(hxdPosNue)*57.3;
    const auto targetTheta = ModelDevelop::Utils::CoordinateHelper::getTheta(hxdPosNue)*57.3;
    missile.launch(targetTheta,targetPsi);

    std::cout<<"dis:"<<missile.targetDis()/1000.0<<"km"<<std::endl;

    for(int i = 0;i<1000/0.005;i++) {
        hxd.update();
        auto hxdVelEcf = ModelDevelop::Utils::CoordinateHelper::nueToEcefVelocity(hxd.velocityNUE(),hxd.lla().x(),hxd.lla().y());
        missile.setTargetLLA(hxd.lla(),hxdVelEcf,false);
        double ret = missile.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
