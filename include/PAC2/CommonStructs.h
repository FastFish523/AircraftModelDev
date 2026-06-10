//
// Created by MikuSoft on 2026/1/27.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//

#pragma once 
#include "Eigen/Dense"
namespace ModelDev::PAC2 {
    struct EigenInfo {
        double mass=0.;
        Eigen::Vector3d P_body= Eigen::Vector3d::Zero();
        Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    };

    struct LosInfo {
        double sigma_elv=0.;
        double sigma_az=0.;
        double sigma_elv_dot=0.;
        double sigma_az_dot=0.;
        double dis_dot=0.;
    };

    struct GCInfo {
        LosInfo losInfo{};
        Eigen::Vector3d acc_cmd_v = Eigen::Vector3d::Zero();
    };

    struct ImuInfo {
        Eigen::Vector3d imu_acc_body= Eigen::Vector3d::Zero();
        Eigen::Vector3d imu_w_xyz_body= Eigen::Vector3d::Zero();
        Eigen::Vector3d imu_ypr= Eigen::Vector3d::Zero();
    };

    struct Derivative {
        double a11 = 0;
        double a12 = 0;
        double a13 = 0;
        double a14 = 0;
        double a15 = 0;
        double a16 = 0;

        double a21 = 0;
        double a22 = 0;
        double a23 = 0;
        double a24 = 0;
        double a25 = 0;
        double a26 = 0;

        double a31 = 0;
        double a32 = 0;
        double a33 = 0;
        double a34 = 0;
        double a35 = 0;
        double a36 = 0;

        double b11 = 0;
        double b12 = 0;
        double b13 = 0;
        double b14 = 0;
        double b15 = 0;
        double b16 = 0;
        double b17 = 0;
        double b18 = 0;

        double b21 = 0;
        double b22 = 0;
        double b23 = 0;
        double b24 = 0;
        double b25 = 0;
        double b26 = 0;
        double b27 = 0;
        double b28 = 0;

        double b31 = 0;
        double b32 = 0;
        double b33 = 0;
        double b34 = 0;
        double b35 = 0;
        double b36 = 0;
        double b37 = 0;
        double b38 = 0;
    };
}