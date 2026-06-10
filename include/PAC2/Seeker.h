//
// Created by MikuSoft on 2026/1/23.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//

#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
#include "CommonStructs.h"
#include "State.h"
#include "Eigen/Core"
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
    class Dll_Export_Import Seeker {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Seeker() = default;

        ~Seeker() = default;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 模拟导引头工作
         * @param targetPosEcf
         * @param targetVelEcf
         * @param state
         * @return
         */
        LosInfo getLOSInfo(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state);

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
