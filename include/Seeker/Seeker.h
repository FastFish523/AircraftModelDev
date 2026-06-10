//
// Created by Administrator on 2026/3/12.
//


#pragma once 
#define PRETTY_FILE_NAME "ModelDevelop/Seeker/Seeker"
#if defined(_WIN32) && !defined(StaticCommonSeeker_Build)
#ifdef SharedCommonSeeker_Build
#define DLL_EXPORT_IMPORT __declspec(dllexport)
#else
#define DLL_EXPORT_IMPORT __declspec(dllimport)
#endif
#else
#define DLL_EXPORT_IMPORT
#endif
// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include  "AttitudeRateSolver.h"

#include "CommonStructs.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "State.h"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/SEEKER/SEEKER"
// endregion

namespace ModelDevelop::SEEKER {
    // class AttitudeRateSolver;
class DLL_EXPORT_IMPORT Seeker {
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
        AttitudeRateSolver _solver;
// endregion

// region Private Methods
    private:
// endregion
    };
}
#undef PRETTY_FILE_NAME