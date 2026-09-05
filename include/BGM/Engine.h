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
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/BGM/BGM"
// endregion

namespace ModelDevelop::BGM {
class Engine {
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
    double _MH=0.7;
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

        /*! @brief Restore the consumable engine state for a new simulation run. */
        void reset();

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
        /*!
         * @brief 一级平均推力
         */
        double F1 = 255 * 1000;
        /*!
         * @brief 二级平均推力
         */
        double F2 = 4.5 * 1000;
        /*!
         * @brief 一级最大推力建立时间
         */
        double dT1_up = 0.1;
        /*!
         * @brief 一级平均推力作用时间
         */
        double T1 = 2;
        /*!
         * @brief 一级推力消退时间
         */
        double dT1_down = 0.1;

        /*!
         * @brief 质量
         */
        double m = 450;
        /*!
         * @brief 比冲
         */
        double Isp0 = 1000.0;
        /*!
         * @brief 比冲
         */
        double Isp1 = 7680.0;
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
#undef PRETTY_FILE_NAME
