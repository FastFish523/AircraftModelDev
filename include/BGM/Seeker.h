//
// Created by Administrator on 2026/3/12.
//


#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "CommonStructs.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "State.h"
#include "functional"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/BGM/BGM"
// endregion

namespace ModelDevelop::BGM{
    using GetLOSInfoFunction = std::function<LosInfo(const Eigen::Vector3d& targetPosEcf,
                                                     const Eigen::Vector3d& targetVelEcf, const State& state)>;

    class Seeker{
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
        void setGetLOSInfoFunction(GetLOSInfoFunction getLOSInfoFunction);
        // endregion
        // region Private Attributes
    private:
        GetLOSInfoFunction myGetLOSInfoFunction;
        // endregion

        // region Private Methods
    private:
        // endregion
    };
}
#undef PRETTY_FILE_NAME
