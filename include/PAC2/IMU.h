//
// Created by MikuSoft on 2026/1/23.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//

#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
#include <Eigen/Core>
#include "CommonStructs.h"
#include "State.h"
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
    class Dll_Export_Import IMU {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        IMU() = default;

        ~IMU() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 获取imu信息
         * @param state 自身状态
         * @param total_acc_ecf ecf系下和力 后续减去重力作为加表输出
         * @return
         */
        ImuInfo getImuInfoBody(const State &state, const Eigen::Vector3d &total_acc_ecf);

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
// endregion

// region Private Methods
    private:
// endregion
    };
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME
