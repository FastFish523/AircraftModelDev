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
#include <Eigen/Core>
#include "Seeker.h"
#include "State.h"
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/SM6/SM6"
// endregion

namespace ModelDevelop::SM6 {
class Guidance {
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
#undef PRETTY_FILE_NAME