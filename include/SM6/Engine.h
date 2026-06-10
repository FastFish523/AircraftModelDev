//
// Created by Administrator on 2026/3/3.
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
#define PRETTY_FILE_NAME "ModelDevelop/SM6/SM6"
// endregion

namespace ModelDevelop::SM6 {
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
         * @brief 1级平均推力
         */
        double F1 = 174 * 1000;
        /*!
         * @brief 2级平均推力
         */
        double F2 = 110 * 1000;
        /*!
         * @brief 3级平均推力
         */
        double F3 = 20* 1000;
        /*!
         * @brief 一级最大推力建立时间
         */
        double dT1_up = 0.5;
        /*!
         * @brief 一级平均推力作用时间
         */
        double T1 = 6;
        /*!
         * @brief 一级推力消退时间
         */
        double dT1_down = 0.5;
        /*!
         * @brief 二级平均推力作用时间
         */
        double T2 = 2;
        /*!
         * @brief 二级推力消退时间
         */
        double dT2_down = 0.5;
        /*!
         * @brief 3级平均推力作用时间
         */
        double T3 = 30;
        /*!
         * @brief 二级推力消退时间
         */
        double dT3_down = 0.1;
        /*!
         * @brief 质量
         */
        double m = 861;
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
#undef PRETTY_FILE_NAME