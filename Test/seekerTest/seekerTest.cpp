//
// Created by MikuSoft on 2026/5/15.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//
#include "ThirdParty/eigen3/Eigen/Core"
#include "Util/CoordinateHelper.h"
int main() {

    Eigen::Vector3d target_position_nue = {1,0.2,1};
    Eigen::Vector3d position_nue        = {0,1,0};
    const auto rel_pos = ((target_position_nue - position_nue)/(target_position_nue - position_nue).norm()).eval();


    std::cout<<"以下程序演示nue系下的视线转弹体系下的转换流程:"<<std::endl;
    std::cout<<"以弹下点nue为坐标原点，弹坐标:\n"<<position_nue<<"\n以弹下点nue为坐标原点，弹坐标:\n"<<target_position_nue<<std::endl;
    const double sigma_elv = ModelDevelop::Utils::CoordinateHelper::getTheta(rel_pos);
    const double sigma_az = ModelDevelop::Utils::CoordinateHelper::getPsi(rel_pos);
    std::cout<<"北天东下的倾角和偏角  ---  sigma_elv(°):"<<sigma_elv*57.3<<"  sigma_az(°):"<<sigma_az*57.3<<std::endl;

    auto qbn                     = ModelDevelop::Utils::CoordinateHelper::euler231ToQuaternion(0*sigma_az*57.3, 0*sigma_elv*57.3, 0);
    double elv_body,az_body;
    auto body_rel_pos = ModelDevelop::Utils::CoordinateHelper::nueToBodyAcceleration(rel_pos,qbn);
    elv_body = ModelDevelop::Utils::CoordinateHelper::getTheta(body_rel_pos);
    az_body = ModelDevelop::Utils::CoordinateHelper::getPsi(body_rel_pos);

    std::cout<<" elv_body(°):"<<elv_body*57.3<<"  az_body(°):"<<az_body*57.3<<std::endl;


    Eigen::Vector3d los{1,0,0};
    const auto theta = elv_body;
    const auto psi = az_body;
    Eigen::Matrix3d Cnv;
    Cnv<<
        cos(theta)*cos(psi),          sin(theta),               -cos(theta)*sin(psi),
        -sin(theta)*cos(psi),         cos(theta),                sin(theta)*sin(psi),
        sin(psi),                      0,                        cos(psi);
    Eigen::Vector3d los_body = Cnv.transpose()*los;
    Eigen::Vector3d los_nue = ModelDevelop::Utils::CoordinateHelper::bodyToNueVector(los_body,qbn);
    auto sigma_elv_new = ModelDevelop::Utils::CoordinateHelper::getTheta(los_nue);
    auto sigma_az_new = ModelDevelop::Utils::CoordinateHelper::getPsi(los_nue);
    std::cout<<"北天东下反计算转出来的的倾角和偏角  ---  sigma_elv_new(°):"<<sigma_elv_new*57.3<<"  sigma_az_new(°):"<<sigma_az_new*57.3<<std::endl;

    return 0;
}