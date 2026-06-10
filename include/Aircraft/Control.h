//
// Created by Administrator on 2026/2/28.
//


#pragma once

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "CommonStructs.h"
#include "State.h"
#include <utility>
#include "ManeuverData.hpp"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Aircraft/Aircraft"
// endregion

namespace ModelDevelop::Aircraft{
    class Control{
        // region USING/FRIEND
    private:
        // endregion

        // region Constructor
    public:
        Control() = default;
        ~Control() = default;
        // endregion

        // region Public Attributes
        bool isCompleted=false;
    public:
        // endregion

        // region Public Methods
    public:
        /*!
         * @brief 六自由度控制
         * @param step
         * @param acc_cmd_v
         * @param state
         * @param totalMass
         * @param p_body
         * @param imu_info
         * @param s 参考面积
         * @param maneuveringType
         * @return rudder moment_body
         */
        std::pair<Eigen::Vector3d, Eigen::Vector3d> P6dof_Control(
            const double&  step, const Eigen::Vector3d& acc_cmd_v,
            const State& state, const double&  totalMass,
            const Eigen::Vector3d& p_body,
            const ImuInfo& imu_info,
            const double&  s,
            const ManeuveringModeType& maneuveringType);

        /*!
         * @brief 最简单的一阶惯性环节
         * @param input
         * @param prev_output
         * @return
         */
        static double firstOrderFilter(double input, double prev_output);

        /*!
         * @brief 限幅方法
         * @param x
         * @param lower
         * @param upper
         * @return
         */
        static double limit(double x, double lower, double upper);
        // endregion

        // region Get/Set选择器
    public:
        /*!
         * @brief 设置舵 控制角速率
         * @param angular_rate_contro 角速率 deg/s
         */
        void setAngularRateControl(const Eigen::Vector3d& angular_rate_control);
        // endregion
        void setManeuveringStageFun(const std::function<ManeuveringStage()>& fun){
            getManeuveringStage=fun;
        }
        // region Private Attributes
    private:
        std::function<ManeuveringStage()> getManeuveringStage=nullptr;
        double alpha_cmd = 0;
        double beta_cmd = 0;
        double pre_ex = 0;
        double pre_ey = 0;
        double pre_ez = 0;
        double ex = 0;
        double ey = 0;
        double ez = 0;
        double iex = 0;
        double iey = 0;
        double iez = 0;

        double last_dx = 0;
        double last_dy = 0;
        double last_dz = 0;
        /*!
         *@brief rudder角速率
         *@param x 滚转舵 deg/s
         *@param y 偏航舵 deg/s
         *@param z 俯仰舵 deg/s
         */
        Eigen::Vector3d m_angular_rate_control={0.0,0.0,0.0};


        // endregion

        // region Private Methods
    private:
        // endregion
    };
}
#undef PRETTY_FILE_NAME
