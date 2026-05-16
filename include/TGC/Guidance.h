//
// Created by 17298 on 2026/4/22.
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
#include <deque>
#include <optional>
#include <utility>
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
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
        GCInfo getMissionGCInfo(double flyTime, double P, double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, double maxLoad,
                                const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex);

        /*!
         * @brief 获取制导控制信息
         * @param state 自身状态
         * @param maxLoad 最大过载
         * @param waypoints
         * @param currentWpIndex
         * @return
        */
        GCInfo getGCInfoRouteL1(const State &state, double maxLoad, const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex);

        void reset();

        /*!
         * @brief l1制导律调用入口
         * @param maxLoad 最大过载
         * @param currentPosEcf 当前位置
         * @param currentVelEcf 当前速度
         * @param waypoints 航点（经度，纬度，高度）
         * @param currentWpIndex 当前航段起点索引（会被更新）
         * @return 侧向加速度指令   巡航高度
         */
        std::pair<double, double> calculateL1Guidance(double maxLoad, const Eigen::Vector3d &currentPosEcf, const Eigen::Vector3d &currentVelEcf,
                                                      const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex);

        /*!
        * @brief 核心L1制导算法 - 修正版
        * @param pos_nue 当前位置[N, E]（忽略天向）
        * @param vel_nue 当前速度[N, E]
        * @param wp_start 航段起点[N, E]
        * @param wp_end 航段终点[N, E]
        * @param L1_distance L1距离
        * @return 横向加速度（m/s²）
        */
        double calculateL1GuidanceNUE(const Eigen::Vector2d &pos_nue, const Eigen::Vector2d &vel_nue, const Eigen::Vector2d &wp_start, const Eigen::Vector2d &wp_end,
                                      double L1_distance = 100.0);

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
         * @brief 上一个预期高度
        */
        double lastDesiredH = 0;
        std::optional<double> boostInitialPsi = std::nullopt;
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
        struct BoostGuidanceInfo {
            Eigen::Vector3d acc_cmd_v{0, 0, 0};
            Eigen::Vector3d tvc_cmd{0, 0, 0};
            double pitch_cmd = 0.0;
            bool pitch_cmd_valid = false;
        };

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

        BoostGuidanceInfo calculateBoostGuidance(double flyTime, double P, double Mass, const Eigen::Vector3d &targetPosEcf, double maxLoad,
                                                 const State &state);

        static double smoothStep(double ratio);
        static double computeBlendRatio(double targetDistance, double startDistance, double endDistance);

    public:
        static double wrapAngle(double angle);
        static double clamp(double value, double minValue, double maxValue);

// endregion
    };
}
#undef PRETTY_FILE_NAME
