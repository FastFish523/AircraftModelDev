//
// Created by MikuSoft on 2026/3/30.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//
#ifndef MANEUVERINGMODE_H
#define MANEUVERINGMODE_H
#include <memory>
#include "IMU.h"
#include "Control.h"

namespace ModelDevelop::Aircraft{
    /*!
     * 转弯度数检查器
     */
    class TurnChecker{
    public:
        /*!
         * @brief 构造
         * @param startYawDeg 起始角度
         * @param dir 方向
         * @param turnDeg 所转度数
         */
        TurnChecker(double startYawDeg, TurnDirection dir, double turnDeg);
        /*!
         * @brief 步进并判断
         * @param currentYawDeg  当前度数
         * @return 是/否
         */
        bool updateAndCheck(double currentYawDeg);
        /*!
         * @brief 获取当前累计度数
         * @return 当前累计
         */
        [[nodiscard]] double getAccumulated() const;
        /*!
         * @brief 重置
         * @param startYawDeg 起始角度
         * @param dir 方向
         * @param turnDeg 所转度数
         */
        void reset(double startYawDeg, TurnDirection dir, double turnDeg);

    private:
        /*!
         * @brief 归一化角度到[-180, 180]
         * @param deg 偏航度数
         * @return 归一化的度数
         */
        static double normalize(double deg);
        /*!
         * @brief 步进计算累计
         * @param curr 当前角度
         * @param prev 上一帧角度
         * @return 本帧累积量
         */
        double calcStep(double curr, double prev) const;
        /*!
         * @brief 累计是否达到目标值
         * @return 是/否
         */
        bool isFinished() const;

    private:
        /*!
         * @brief 保存上一帧角度
         */
        double m_prev_yaw;
        /*!
         * @brief 总累计
         */
        double m_total;
        /*!
         * @brief 需要偏转量
         */
        double m_turn;
        /*!
         * @brief 偏转方向
         */
        TurnDirection m_direction;
    };

    /*!
     * 平飞
     */
    class LevelFight : public BaseManeuvering{
    public:
        void init(const State& state, std::string para) override;
        RESULT update(const State& state) override;

    private:
        /*!
         * 平飞目标高度
         */
        double m_targetHeight = 0.0;
    };

    /*!
     * 升降
     */
    class UpAndDown : public BaseManeuvering{
    public:
        void init(const State& state, std::string para) override;
        RESULT update(const State& state) override;

    private:
        /*!
         * 目标高度
         */
        double m_targetHeight = 0.0;
    };

    /*!
     * L型制导
     */
    class L : public BaseManeuvering{
    public:
        ~L() override{ m_turnChecker = nullptr; };
        void init(const State& state, std::string para) override;
        RESULT update(const State& state) override;

    private:
        /*!
         * @brief 控高度
         */
        double m_targetHeight = 0.0;
        /*!
         * @brief 角度检测器
         */
        std::unique_ptr<TurnChecker> m_turnChecker = nullptr;
        /*!
         * @brief 偏转方向
         */
        TurnDirection m_direction = TurnDirection::Left;
    };

    /*!
     * @brief 圆周运动
     */
    class Circle : public BaseManeuvering{
    public:
        ~Circle() override{ m_turnChecker = nullptr; };
        void init(const State& state, std::string para) override;
        RESULT update(const State& state) override;

    private:
        /*!
         * @brief 控高度
         */
        double m_targetHeight = 0.0;
        /*!
         * @brief 角度检测器
         */
        std::unique_ptr<TurnChecker> m_turnChecker = nullptr;
        /*!
         * @brief 偏转方向
         */
        TurnDirection m_direction = TurnDirection::Left;
        /*!
         * @brief 圈数
         */
        double m_number = 1;
    };

    /*!
     * @brief 水平S
     */
    class SLevel : public BaseManeuvering{
    public:
        ~SLevel(){ m_turnChecker = nullptr; };
        void init(const State& state, std::string para) override;
        RESULT update(const State& state) override;

    private:
        /*!
         * @brief 控高度
         */
        double m_targetHeight = 0.0;
        /*!
         * @brief 角度检测器
         */
        std::unique_ptr<TurnChecker> m_turnChecker = nullptr;
        /*!
         * @brief 偏转方向
         */
        TurnDirection m_direction = TurnDirection::Left;
        constexpr static double turnDeg = 180;
        bool isFirst = true;
    };

    /*!
     * 垂直S
     */
    class SVertical : public BaseManeuvering{
    public:
        void init(const State& state, std::string para) override;
        RESULT update(const State& state) override;

    private:
        /*!
         * 初始高度
         */
        double m_initialHeight = 0.0;
        /*!
         * 机动已经持续时间
         */
        double m_durationTime = 0.0;
        /*!
         * 计划持续时间
         */
        double m_minDurationTime = 0.0;
    };

    /*!
     * 垂直S
     */
    class Somersault : public BaseManeuvering{
    public:
        void init(const State& state, std::string para) override;
        RESULT update(const State& state) override;

    private:
        double lastPitch = 0.0;
        double pitchChangeCum = 0.0;
    };

    class SplitS : public BaseManeuvering{
    public:
        void init(const State& state, std::string para) override;
        RESULT update(const State& state) override;

    private:
        double lastPitch = 0.0;
        double pitchChangeCum = 0.0;
    };

    /*!
     * 机动控制模块
     */
    class ManeuveringMode{
    public:
        ManeuveringMode() = default;
        ~ManeuveringMode() = default;
        /*!
         * @brief 设置机动
         * @param _type 机动类型
         * @param _paraStr 机动参数（需要序列化成字符串）
         * @param state 实体当前状态信息
         */
        auto setManeuvering(ManeuveringModeType _type, const std::string& _paraStr,
                            const State& state) -> void;
        /*!
         * @brief 步进
         * @param step
         * @param state
         * @param totalMass
         * @param p_body
         * @param imu_info
         * @param s
         * @return
         */
        auto update(double step, const State& state, const double totalMass,
                    const Eigen::Vector3d& p_body, const ImuInfo& imu_info,
                    const double& s) -> std::pair<Eigen::Vector3d, Eigen::Vector3d>;
        /*!
         * 获取当前机动类型
         * @return 当前机动类型
         */
        auto getManeuveringModeType() const -> ManeuveringModeType;

    private:
        ManeuveringModeType m_currentType = ManeuveringModeType::LevelFlight;
        std::unique_ptr<BaseManeuvering> _maneuveringPtr = nullptr;
        /*!
         * @brief 控制系统
         */
        Control _control{};
    };
}
#endif
