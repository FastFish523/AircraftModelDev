//
// Created by MikuSoft on 2026/3/16.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//
#include "SEEKER/AttitudeRateSolver.h"

namespace ModelDevelop::SEEKER{
    void AttitudeRateSolver::update(double q_d_p, double q_d_y){
        if (isFirstUpdate){
            td_pitch.reset(q_d_p);
            td_yaw.reset(q_d_y);
            isFirstUpdate = false;
        }
        td_pitch.update(q_d_p);
        td_yaw.update(q_d_y);
        rate_pitch_cmd = td_pitch.getRate();
        rate_yaw_cmd = td_yaw.getRate();
        // std::cout << "俯仰角速率" << rate_pitch_cmd << "偏航角速率" << rate_yaw_cmd << std::endl;
    }
    void AttitudeRateSolver::SingleChannelTD::reset(double initial_v){
        x1 = initial_v; // 将跟踪信号直接设为初始值，消除初始误差
        x2 = 0.0; // 速率通常初始化为 0，假设初始时刻静止
    }

    void AttitudeRateSolver::SingleChannelTD::update(double v){
        constexpr double d = r * h;
        constexpr double d0 = h * d;
        constexpr double dt = 1 / fs;
        const double y = x1 - v + h * x2;
        const double a0 = std::sqrt(d * d + 8.0 * r * std::fabs(y));
        const double a = std::fabs(y) > d0
                             ? x2 + (a0 - d) / 2.0 * sign(y)
                             : x2 + y / h;
        const double abs_a = std::fabs(a);
        const double fhan = (d != 0) ? (-r * sign(a) * std::min(std::fabs(a), d) / d) : 0.0;
        x2 = x2 + fhan * dt;
        x1 = x1 + x2 * dt;
    }

    double AttitudeRateSolver::SingleChannelTD::sign(const double& x){ return (x > 0) ? 1.0 : ((x < 0) ? -1.0 : 0.0); }
}
