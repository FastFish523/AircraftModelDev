#include <deque>
#include <filesystem>
#include <iostream>
#include "include/PAC2/Missile.h"


int main()
{
    double step=0.001;
    // const Eigen::Vector3d targetLLA    = {119.6,40,12000};
    // Eigen::Vector3d pac2LLA            = {120,40,2};
    Eigen::Vector3d targetLLA = {120, 40, 10 * 1000};
    Eigen::Vector3d pac2LLA = {120, 40.14, 2};
    const Eigen::Vector3d targetPosEcf = ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetLLA);
    const Eigen::Vector3d targetPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,pac2LLA.x(),pac2LLA.y());

    ModelDev::PAC2::Missile pac2;
    pac2.init(step,pac2LLA);
    pac2.setTargetEcf(targetPosEcf,{0,0,0});
    const auto targetTheta = ModelDevelop::Utils::CoordinateHelper::getTheta(targetPosNue)*57.3;
    const auto targetPsi = ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue)*57.3;
    pac2.launch(targetTheta,targetPsi-10);

    for(int i = 0;i<100/step;i++) {
        double ret = pac2.update();
        if (ret>0) {
            std::cout << "terminal_dis:" << ret << std::endl;
            break;
        }
    }
    return 0;
}
