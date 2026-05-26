//
// Created by 17298 on 2026/5/16.
//
// 助推段可调参数配置
// 修改此文件即可调整助推段程序角、TVC限幅、终端修正等参数，无需深入 Guidance.cpp

#pragma once

namespace ModelDevelop::TGC::BoostConfig {

// ============================================================
// 发射参数 (度)
// ============================================================
constexpr double LAUNCH_THETA_F             = 45.0;   // 发射倾角（发射系相对水平面倾角）

// ============================================================
// 助推段程序角设计 (度) —— 各阶段俯仰角指令
// ============================================================
constexpr double STAGE1_START_BLEND_THETA   = 45.0;   // 一级混合起始角
constexpr double STAGE1_END_TARGET_THETA    = 45.0;   // 一级结束目标角

constexpr double STAGE2_START_BLEND_THETA   = 45.0;   // 二级混合起始角
constexpr double STAGE2_END_TARGET_THETA    = 15.0;   // 二级结束目标角

constexpr double STAGE3_SPLIT_START_THETA   = 15.0;   // 三级分段起始角
constexpr double STAGE3_SPLIT_END_THETA     = -40.0;    // 三级分段结束目标角

constexpr double STAGE3_FINAL_START_THETA   = -40.0;    // 三级末段程序角起始
constexpr double STAGE3_FINAL_END_THETA     = 10.0;    // 三级末段程序角结束（燃尽时刻）

// ============================================================
// TVC 摆角限幅 (度)
// ============================================================
constexpr double TVC_LIMIT_STAGE1           = 60.0;    // 一级
constexpr double TVC_LIMIT_STAGE2           = 60.0;    // 二级
constexpr double TVC_LIMIT_STAGE3_SPLIT     = 60.0;   // 三级分段阶段
constexpr double TVC_LIMIT_STAGE3_FINAL     = 60.0;    // 三级末段

// ============================================================
// 时序参数 (秒)
// ============================================================
constexpr double THIRD_STAGE_SPLIT_TIME     = 150.0;  // 三级从程序制导向终端修正切换的时间

// ============================================================
// 目标 Burnout 状态
// ============================================================
constexpr double TARGET_BURNOUT_ALTITUDE    = 130000.0; // 目标燃尽高度 (m)
constexpr double TARGET_BURNOUT_VELOCITY    = 6000.0;   // 目标燃尽速度 (m/s)

// ============================================================
// 终端修正增益
// ============================================================
constexpr double TERMINAL_ALTITUDE_GAIN     = 2.0;  // 高度误差增益
constexpr double TERMINAL_VELOCITY_GAIN     = 1.0e-5;  // 速度误差增益
constexpr double TERMINAL_CORRECTION_LIMIT  = 2.0;     // 终端修正角限幅 (度)

// ============================================================
// 助推段 PD 反馈增益
// ============================================================
constexpr double BOOST_SPEED_THETA_GAIN     = 1.6;     // speed * thetaError 系数
constexpr double BOOST_DAMPING_GAIN         = 0.35;    // 阻尼系数 (Vy 反馈)
constexpr double BOOST_PSI_GAIN             = 1.2;     // speed * psiError 系数

}  // namespace ModelDevelop::TGC::BoostConfig
