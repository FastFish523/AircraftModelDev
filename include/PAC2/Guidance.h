//
// Created by MikuSoft on 2026/1/26.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//

#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
#include <Eigen/Core>
#include "CommonStructs.h"
#include "Seeker.h"
#include "State.h"
// endregion
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDev/Model/GuidanceAndControl"
#if defined(_WIN32) && !defined(StaticModel_Build)
#ifdef SharedModel_Build
#define Dll_Export_Import __declspec(dllexport)
#else
#define Dll_Export_Import __declspec(dllimport)
#endif
#else
#define Dll_Export_Import
#endif

// endregion

namespace ModelDev::PAC2 {
    class Dll_Export_Import Guidance {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Guidance() = default;

        ~Guidance() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 获取制导控制信息
         * @param flyTime 飞行时间
         * @param P 推力牛
         * @param Mass 质量 千克
         * @param targetPosEcf 目标位置
         * @param targetVelEcf 目标速度
         * @param state 自身状态
         * @param maxLoad 最大过载
         * @return
         */
        GCInfo getGCInfo(double flyTime, double P, double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, double maxLoad);

        /*!
         * @brief 兰伯特1
         * @param r_m 起始点ecf坐标
         * @param r_pip 落点ecf坐标
         * @param T_pip 飞行时间 s
         * @param vd_m 发射速度向量ecf
         * @param Range 射程
         */
        void Lambert_Resolve_Dv1(const Eigen::Vector3d& r_m, const Eigen::Vector3d &r_pip, double &T_pip, double vd_m[3], double &Range) const;

        /*!
         * @brief 兰伯特
         * @param r_m 起始点ecf坐标
         * @param r_pip 落点ecf坐标
         * @param T_pip 飞行时间 s
         * @param vd_m 发射速度向量ecf
         * @param Range 射程
         */
        void Lambert_Resolve_Dv(const Eigen::Vector3d& r_m, const Eigen::Vector3d &r_pip, double &T_pip, double vd_m[3], double &Range) const;

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
        /*!
         * @brief 导引头
         */
        Seeker _seeker{};
        /*!
         * @brief 兰伯特制导参数
         */
        const double c_dPi       = 3.1415926535897932384626433832795;
        /*!
         * @brief 兰伯特制导参数
         */
        const double D2R         = c_dPi / 180.0;
        /*!
         * @brief 兰伯特制导参数
         */
        const double R2D         = 180.0 / c_dPi;
        /*!
         * @brief 兰伯特制导参数
         */
        const double av          = 340.0;
        /*!
         * @brief 兰伯特制导参数
         */
        const double earth_ae    = 6371004.0;
        /*!
         * @brief 兰伯特制导参数
         */
        const double earth_g0    = 9.80665;
        /*!
         * @brief 兰伯特制导参数
         */
        const double earth_omega = 7.292115E-5;
        /*!
         * @brief 兰伯特制导参数
         */
        const double c_dMiu      = 3.986004418e14;
// endregion

// region Private Methods
    private:
        /*!
         * 获取视线角速率信息
         * @param targetPosEcf
         * @param targetVelEcf
         * @param state
         * @return
         */
        static LosInfo getLOSInfo(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state);

        /*!
         * @brief 比例导引法
         * @param theta 速度倾角 弧度
         * @param sigma_az_dot 视线偏角变化率 弧度/秒
         * @param sigma_elv_dot 视线倾角变化率 弧度/秒
         * @param dis_dot 弹目距离变化率 m/s
         * @return
         */
        static Eigen::Vector3d guidance_pn(double theta, double sigma_az_dot, double sigma_elv_dot, double dis_dot);
// endregion
    };
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME
