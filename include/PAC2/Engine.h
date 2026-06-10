//
// Created by MikuSoft on 2026/1/23.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//

#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
#include <State.h>

#include "CommonStructs.h"
#include "Eigen/Core"
#include "Eigen/Dense"
// endregion
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDev/Model/PAC2"
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
    class Dll_Export_Import Engine {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Engine() = default;

        ~Engine() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 获取发动机推力质量信息
         * @param simStep 仿真步长
         * @param flyTime 飞行时间
         * @param state
         * @param dx 喷管摆角 弧度
         * @param dy 喷管摆角 弧度
         * @param dz 喷管摆角 弧度
         * @return
         */
        EigenInfo getEigenInfo(double simStep, double flyTime, const State& state, double dx = 0, double dy = 0, double dz = 0);

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
        /*!
         * @brief 一级平均推力
         */
        double F1 = 220 * 1000;
        /*!
         * @brief 二级平均推力
         */
        double F2 = 135 * 1000;
        /*!
         * @brief 一级最大推力建立时间
         */
        double dT1_up = 0.5;
        /*!
         * @brief 一级平均推力作用时间
         */
        double T1 = 3;
        /*!
         * @brief 一级推力消退时间
         */
        double dT1_down = 0.5;
        /*!
         * @brief 二级平均推力作用时间
         */
        double T2 = 6.9;
        /*!
         * @brief 二级推力消退时间
         */
        double dT2_down = 0.1;
        /*!
         * @brief 质量
         */
        double m = 726;
        /*!
         * @brief 比冲
         */
        double Isp = 245.0;
        /*!
         * @brief 额外转动惯量
         */
        double jx = 0;
        /*!
         * @brief 额外转动惯量
         */
        double jy = 0;
        /*!
         * @brief 额外转动惯量
         */
        double jz = 0;

// endregion

// region Private Methods
    private:
// endregion
    };
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME
