//
// Created by MikuSoft on 2026/1/23.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//

#pragma once

// region Include
// region STL
// endregion
// region ThirdParty

// endregion
// region Self
#include <deque>
#include <filesystem>
#include <iostream>
#include <optional>
#include "CommonStructs.h"
#include "Control.h"
#include "CoordinateHelper.h"
#include "Engine.h"
#include "FileSaver.h"
#include "Guidance.h"
#include "IMU.h"
#include "Kinematics.h"
#include "Eigen/Core"

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
    class Dll_Export_Import Missile {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        Missile();

        ~Missile();

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        /*!
         * @brief 模型初始化
         * @param step  仿真步长 秒
         * @param lla 经纬高 度 米
         */
        void init(double step, const Eigen::Vector3d &lla);

        /*!
         * @brief 发射
         * @param theta_f_d 发射倾角 度
         * @param psi_f_d 发射偏角 度
         */
        void launch(double theta_f_d, double psi_f_d);

        /*!
         * @brief 设置目标
         * @param targetPosEcf 目标位置
         * @param targetVelEcf 目标速度
         */
        void setTargetEcf(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf);

        /*!
         * @brief 设置目标
         * @param targetPosLLa 目标位置
         * @param targetVelEcf 目标速度
         */
        void setTargetLLA(const Eigen::Vector3d &targetPosLLa, const Eigen::Vector3d &targetVelEcf);

        /*!
         * @brief 仿真更新
         */
        double update();

// endregion

// region Get/Set选择器
    public:
        /*!
         * 获取飞行时间 秒
         * @return
         */
        [[nodiscard]]
        double flyTime() const ;

        /*!
         * @brief 获取自身发射系nue位置
         * @return
         */
        Eigen::Vector3d positionLaunchNUE() const ;

        /*!
         * @brief 获取标量速度
         * @return
         */
        [[nodiscard]]
        double V() const ;

        /*!
         * @brief 获取标量速度ma
         * @return
         */
        [[nodiscard]]
        double Ma() const ;

        /*!
         * @brief 获取自身欧拉角 度
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d attitudeEuler() const ;

        /*!
         * @brief 获取速度倾角 度
         * @return
         */
        [[nodiscard]]
        double velocityTheta() const ;

        /*!
         * @brief 获取速度偏角 度
         * @return
         */
        [[nodiscard]]
        double velocityPsi() const ;

        /*!
         * @brief 获取攻角 度
         * @return
         */
        [[nodiscard]]
        double alpha() const ;

        /*!
         * @brief 获取侧滑角 度
         * @return
         */
        [[nodiscard]]
        double beta() const ;

        /*!
         * @brief 获取体系加速度 m/s2
         * @return
         */
        [[nodiscard]] Eigen::Vector3d accelerationBody() const ;

        /*
         ** @brief 获取体系角速度 弧度/s
         ** @return
         **/
        [[nodiscard]] Eigen::Vector3d w_xyz() const ;

        /*
         * @brief 获取质量 千克
         * @return
         */
        [[nodiscard]]
        double mass() const ;

        /*
         * @brief 获取推力 牛
         * @return
         */
        [[nodiscard]]
        double P() const ;

        /*
         * @brief 获取目标相对发射系 nue位置
         * @return
         */
        Eigen::Vector3d targetPositionLaunchNUE() const ;

        /*
         * @brief 获取舵偏 度
         * @return
         */
        [[nodiscard]] Eigen::Vector3d rudder() const ;

        /*
         * @brief 获取nue 速度 米/s
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d velocityNUE() const ;

        /*
         * @brief 获取经纬高
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d lla() const ;

        /*
         * @brief 获取弹目距离 米
         * @return
         */
        [[nodiscard]]
        double targetDis() const ;

        /*
         * @brief 获取体法向指令加速度
         * @return
         */
        [[nodiscard]] auto acc_cmd_b_y() const -> double ;

        /*
        * @brief 获取体侧向指令加速度
        * @return
        */
        [[nodiscard]] auto acc_cmd_b_z() const -> double ;

        /*!
         * @brief 获取视线倾角,类别速度倾角
         * @return
         */
        [[nodiscard]] auto sigmaElv() const -> double ;

        /*!
         * @brief 获取视线倾角变化率
         * @return
         */
        [[nodiscard]] auto sigmaElvDot() const -> double ;

        /*!
        * @brief 获取视线偏角，类别速度偏角
        * @return
        */
        [[nodiscard]] auto sigmaAz() const -> double ;

        /*!
        * @brief 获取视线偏角变化率
        * @return
        */
        [[nodiscard]] auto sigmaAzDot() const -> double ;

        /*!
         * @brief 气动导数计算
         * @return
         */
        [[nodiscard]]
        Derivative derivative() const;

// endregion

// region Private Attributes
    private:
        /*!
         * @brief 仿真步长
         */
        double _step = 0.005;
        /*!
         * @brief 飞行时间
         */
        double _flyTime = 0.0;
        /*!
         * @brief 发射标识
         */
        bool _launchFlag = false;

        /*!
         * @brief 自身状态
         */
        State _state{};
        /*!
         * @brief 自身质量（不包含发动机）
         */
        double _mass = 0;
        /*!
         * @brief 参考面积
         */
        double _s = 0;
        /*!
         * @brief 参考长度
         */
        double _l = 0;
        /*!
         * @brief 参考翼展
         */
        double _b = 0;

        /*!
         * @brief 总质量（包含发动机）
         */
        double _totalMass = 0;
        /*!
         * @brief 最大过载
         */
        double _maxLoad = 0;
        /*!
         * @brief 转动惯量矩阵
         */
        Eigen::Matrix3d _inertia{};

        /*!
         * @brief 发射点经纬高
         */
        Eigen::Vector3d _launchLLA{};
        /*!
         * @brief 目标位置
         */
        std::optional<Eigen::Vector3d> _targetPosEcf = std::nullopt;
        /*!
         * @brief 目标速度
         */
        Eigen::Vector3d _targetVelEcf{};
        /*!
         * @brief 体法向加速度指令
         */
        double _acc_cmd_b_y = 0;
        /*!
         * @brief 体侧向加速度指令
         */
        double _acc_cmd_b_z = 0;

        /*!
         * @brief 视线倾角
         */
        double _sigma_elv = 0;
        /*!
         * @brief 视线倾角变化率
         */
        double _sigma_elv_dot = 0;
        /*!
            * @brief 视线偏角
            */
        double _sigma_az = 0;
        /*!
         * @brief 视线偏角变化率
         */
        double _sigma_az_dot = 0;
        /*!
         * @brief 舵偏
         */
        Eigen::Vector3d _rudder{0, 0, 0};
        /*!
         * @brief 弹体系推力向量
         */
        Eigen::Vector3d _p_body{0, 0, 0};
        /*!
         * @brief 弹体系直接力矩矢量
         */
        Eigen::Vector3d _m_body{0, 0, 0};
        /*!
         * @brief 惯组输出
         */
        ImuInfo _imu_info{};

        /*!
         * @brief 运动学模块
         */
        ModelDevelop::Utils::Kinematics _kinematics{};

        /*!
         * @brief 发动机
         */
        Engine _engine{};
        /*!
         * @brief 制导系统
         */
        Guidance _guidance{};
        /*!
         * @brief 控制系统
         */
        Control _control{};
        /*!
         * @brief 惯组
         */
        IMU _imu{};

        /*!
         * @brief 接收单目距离的队列
         */
        std::deque<double> distance_deque{};

        /*!
         * @brief 文件保存器
         */
        std::shared_ptr<FileSaver> _fileSaver = nullptr;

// endregion

// region Private Methods
    private:
        /*!
         * @brief 四阶龙格库塔
         * @param _rudder
         * @param P_body
         * @param M_body
         * @return
         */
        Eigen::Vector3d rk4(const Eigen::Vector3d &_rudder, const Eigen::Vector3d &P_body, const Eigen::Vector3d &M_body);

// endregion
    };
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME
