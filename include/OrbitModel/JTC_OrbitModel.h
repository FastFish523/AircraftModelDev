#pragma once
#include <CoordinateHelper.h>

#include "Datetime.h"
#include "FileSaver.h"
#include "OrbitKit.h"
#include "Satellite.h"
#include "stdafx.h"
#include "Tools.h"
#include "ThirdParty/eigen3/Eigen/Dense"
#include "Util/State.h"
// region Define
#if defined(_WIN32) && !defined(StaticOrbitModel_Build)
#ifdef SharedOrbitModel_Build
#define DLL_EXPORT_IMPORT __declspec(dllexport)
#else
#define DLL_EXPORT_IMPORT __declspec(dllimport)
#endif
#else
#define DLL_EXPORT_IMPORT
#endif
// endregion


namespace JTC_Basic_OrbitModel {
    /*!
     * 轨道类型
     */
    enum OrbitTypeEnum {
        /*位置速度*/
        TwoBody4Cartesian = 0, //位置速度
        /*轨道根数*/
        TwoBody4Classical, //轨道根数
        /*TLE*/
        SGP4, //TLE
    };

    /*!
     * 轨道模型
     */
    class DLL_EXPORT_IMPORT JTC_OrbitModel {
    public:
        JTC_OrbitModel(const string& tle_1, const string &tle_2);

        /*轨道根数构造*/
        JTC_OrbitModel(int year, int month, int day, int hour, int minute, double second,
                       double semi_majorAxis, double eccentricity, double inclination, double argOfPerigee, double rann,
                       double meananom);


        /* 计算给定时刻的位置速度
        返回值（18个数据，地惯系下位置速度，地固系下位置速度，经纬高位置速度）*/
        void update(int year, int month, int day, int hour, int minute, double second, double step);

//        /*!
//            * 获取飞行时间 秒
//            * @return
//            */
        [[nodiscard]]
        double flyTime() const {
            return _flyTime;
        }

        /*!
         * @brief 获取自身发射系nue位置
         * @return
         */
        Eigen::Vector3d positionLaunchNUE() const {
            return {};
        }

        /*!
         * @brief 获取标量速度
         * @return
         */
        [[nodiscard]]
        double V() const {
            return _state.velEcf.norm();
        }

        /*!
         * @brief 获取标量速度ma
         * @return
         */
        [[nodiscard]]
        double Ma() const {
            return _state.velEcf.norm() / 340.0;
        }

        /*!
         * @brief 获取自身欧拉角 度
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d attitudeEuler() const {
            return {};
        }

        /*!
         * @brief 获取速度倾角 度
         * @return
         */
        [[nodiscard]]
        double velocityTheta() const {
            return {};
        }

        /*!
         * @brief 获取速度偏角 度
         * @return
         */
        [[nodiscard]]
        double velocityPsi() const {
            return {};
        }

        /*!
         * @brief 获取攻角 度
         * @return
         */
        [[nodiscard]]
        double alpha() const {
            return {};
        }

        /*!
         * @brief 获取侧滑角 度
         * @return
         */
        [[nodiscard]]
        double beta() const {
            return {};
        }

        /*!
         * @brief 获取体系加速度 m/s2
         * @return
         */
        [[nodiscard]] Eigen::Vector3d accelerationBody() const {
            return {};
        }

        /*
         ** @brief 获取体系角速度 弧度/s
         ** @return
         **/
        [[nodiscard]] Eigen::Vector3d w_xyz() const {
            return {};
        }

        /*
         * @brief 获取质量 千克
         * @return
         */
        [[nodiscard]]
        double mass() const {
            return {};
        }

        /*
         * @brief 获取推力 牛
         * @return
         */
        [[nodiscard]]
        double P() const {
            return {};
        }

        /*
         * @brief 获取目标相对发射系 nue位置
         * @return
         */
        Eigen::Vector3d targetPositionLaunchNUE() const {
            return {};
        }

        /*
         * @brief 获取舵偏 度
         * @return
         */
        [[nodiscard]] Eigen::Vector3d rudder() const {
            return {};
        }

        /*
         * @brief 获取nue 速度 米/s
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d velocityNUE() const {
            Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(_state.posEcf);
            return ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(_state.velEcf, lla.x(), lla.y());
        }

        /*
         * @brief 获取经纬高
         * @return
         */
        [[nodiscard]]
        Eigen::Vector3d lla() const {
            Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(_state.posEcf);
            return lla;
        }

        /*
         * @brief 获取弹目距离 米
         * @return
         */
        [[nodiscard]]
        double targetDis() const {
            return {};
        }

        /*
         * @brief 获取体法向指令加速度
         * @return
         */
        [[nodiscard]] auto acc_cmd_b_y() const -> double {
            return {};
        }

        /*
        * @brief 获取体侧向指令加速度
        * @return
        */
        [[nodiscard]] auto acc_cmd_b_z() const -> double {
            return {};
        }

        /*!
         * @brief 获取视线倾角,类别速度倾角
         * @return
         */
        [[nodiscard]] auto sigmaElv() const -> double {
            return {};
        }

        /*!
         * @brief 获取视线倾角变化率
         * @return
         */
        [[nodiscard]] auto sigmaElvDot() const -> double {
            return {};
        }

        /*!
        * @brief 获取视线偏角，类别速度偏角
        * @return
        */
        [[nodiscard]] auto sigmaAz() const -> double {
            return {};
        }

        /*!
        * @brief 获取视线偏角变化率
        * @return
        */
        [[nodiscard]] auto sigmaAzDot() const -> double {
            return {};
        }

    private:
        COrbitKit _orbitKit;
        SATELLITE _sat;
        Satellite _satT1;
        Tools _tools;
        /*!
         * @brief 自身状态
         */
        State _state{};
        /*
         * 时间转换器
         */
        Datetime _timeConvertor;
        /*
         * 轨道类型
         */
        OrbitTypeEnum _orbitType;
        /*
         * 历元时刻
         */
        _datetime _epoch;
        /*
         * 六根数
         */
        coe _coe;
        /*
         * 地惯系下位置：千米
         */
        double _posEci[3];
        /*
         * 地惯系下速度：千米/秒
         */
        double _velEci[3];
        /*
         * 地固系下位置：千米
         */
        double _posEcf[3];
        /*
         * 地固系下速度：千米/秒
         */
        double _velEcf[3];
        /*
         * 经纬高（km）
         */
        double _lla[3];
        /*!
         * @brief 飞行时间
         */
        double _flyTime = 0;

        /*!
         * @brief 文件保存器
        */
        std::shared_ptr<FileSaver> _fileSaver = nullptr;
    };
}
#undef DLL_Export_Import
