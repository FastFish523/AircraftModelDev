//
// Created by Administrator on 2026/1/30.
//

#include <iostream>
#include <iomanip>

#include "GPI/Missile.h"
#include "Util/CoordinateHelper.h"

int main()
{
    const double step = 0.001;

    // 目标输入：起始经纬高、速度大小、方向点经纬高。
    const Eigen::Vector3d targetStartLLA       = {-110, 37, 30 * 1000};
    const double targetSpeed                   = 500.0; // m/s
    const Eigen::Vector3d targetDestinationLLA = {-115, 38, 35 * 1000};
    Eigen::Vector3d gpiLLA                     = {-105.19, 37.05, 30 * 1000};

    Eigen::Vector3d targetPosEcf =
        ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetStartLLA);
    const Eigen::Vector3d targetDestinationPosEcf =
        ModelDevelop::Utils::CoordinateHelper::llaToEcef(targetDestinationLLA);
    const Eigen::Vector3d targetDirectionEcf =
        targetDestinationPosEcf - targetPosEcf;

    if (targetDirectionEcf.norm() == 0.0 || targetSpeed <= 0.0) {
        std::cerr << "Target direction must be non-zero and speed must be positive.\n";
        return 1;
    }

    const Eigen::Vector3d targetVelEcf =
        targetDirectionEcf.normalized() * targetSpeed;
    const Eigen::Vector3d targetPosNue =
        ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(
            targetPosEcf, gpiLLA.x(), gpiLLA.y());

    ModelDevelop::GPI::Missile gpi;
    gpi.init(step, gpiLLA);
    gpi.setTargetEcf(targetPosEcf, targetVelEcf);
    const auto targetPsi =
        ModelDevelop::Utils::CoordinateHelper::getPsi(targetPosNue) * 57.3;

    // gpi发射输入：发射倾角、发射偏角、速度。
    gpi.launch(0, targetPsi, 3000);

    const int statusIntervalSteps = static_cast<int>(10.0 / step);
    for (int i = 0; i < 1000 / step; ++i) {
        targetPosEcf += targetVelEcf * step;
        gpi.setTargetEcf(targetPosEcf, targetVelEcf);
        const double ret2 = gpi.update();

        if ((i + 1) % statusIntervalSteps == 0) {
            const Eigen::Vector3d targetLLA =
                ModelDevelop::Utils::CoordinateHelper::ecefToLla(targetPosEcf);
            const Eigen::Vector3d currentGpiLLA = gpi.lla();
            std::cout << std::fixed << std::setprecision(3)
                      << "time=" << gpi.flyTime() << " s"
                      << ", target_LLA=[" << targetLLA.transpose() << "]"
                      << ", target_speed=" << targetVelEcf.norm() << " m/s"
                      << ", gpi_LLA=[" << currentGpiLLA.transpose() << "]"
                      << ", gpi_speed=" << gpi.V() << " m/s"
                      << ", distance=" << gpi.targetDis() / 1000.0 << " km"
                      << std::endl;
        }

        if (ret2 > 0) {
            const Eigen::Vector3d targetLLA =
                ModelDevelop::Utils::CoordinateHelper::ecefToLla(targetPosEcf);
            const Eigen::Vector3d currentGpiLLA = gpi.lla();
            std::cout << std::fixed << std::setprecision(3)
                      << "time=" << gpi.flyTime() << " s"
                      << ", target_LLA=[" << targetLLA.transpose() << "]"
                      << ", target_speed=" << targetVelEcf.norm() << " m/s"
                      << ", gpi_LLA=[" << currentGpiLLA.transpose() << "]"
                      << ", gpi_speed=" << gpi.V() << " m/s"
                      << ", distance=" << ret2/1000.0 << " km"
                      << std::endl;
            std::cout << "terminal_dis2:" << ret2 << " m" << std::endl;
            break;
        }
    }
    return 0;
}
