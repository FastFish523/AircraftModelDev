//
// Created by Administrator on 2026/1/30.
//

#include <iostream>
#include <HXD3530/Control.h>
#include <HXD3530/Missile.h>

#include "GPI/Missile.h"
#include "HXD3530/HXD.h"

int main()
{
    double step = 0.001;
    const Eigen::Vector3d hxdLLA    = {121,24,35*1000};
    const Eigen::Vector3d hxdTargetLLA    = {110,24,35*1000};

    Eigen::Vector3d gpiLLA            = {117,24,35*1000};

    const Eigen::Vector3d hxdTargetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(hxdTargetLLA);
    const Eigen::Vector3d hxdTargetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(hxdTargetPosEcf,hxdLLA.x(),hxdLLA.y());

    const Eigen::Vector3d hxdPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(hxdLLA);
    const Eigen::Vector3d hxdPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(hxdPosEcf,gpiLLA.x(),gpiLLA.y());

    ModelDevelop::HXD3530::Missile hxd;
    hxd.init(step,hxdLLA);
    hxd.setTargetEcf(hxdTargetPosEcf,{0,0,0});
    const auto hxdTargetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(hxdTargetPosNue)*57.3;
    hxd.launch(0,hxdTargetPsi);

    ModelDevelop::GPI::Missile gpi;
    gpi.init(step,gpiLLA);
    gpi.setTargetEcf(hxdPosEcf,{0,0,0});

    const auto hxdPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(hxdPosNue)*57.3;
    gpi.launch(0,hxdPsi,3000);

    for(int i = 0;i<1000/step;i++) {
        hxd.update();
        auto hxdVelEcf = ModelDevelop::Utils::CoordinateHelper::nueToEcefVelocity(hxd.velocityNUE(),hxd.lla().x(),hxd.lla().y());
        gpi.setTargetLLA(hxd.lla(),hxdVelEcf);
        const double ret = gpi.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
