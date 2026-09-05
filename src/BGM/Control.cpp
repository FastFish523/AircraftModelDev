//
// Created by Administrator on 2026/3/12.
//

// region Include
// region STL
#include <cmath>
#include <stdexcept>
// endregion
// region ThirdParty
// endregion
// region Self
#include "BGM/Control.h"
#include "Aerodynamics.h"
#include "CoordinateHelper.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/BGM/BGM"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::BGM {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
    void Control::configure(const ControlModuleConfig &config) {
        switch (config.module) {
            case ControlModule::P6dofPi:
            case ControlModule::P6dofP:
                break;
            default:
                throw std::invalid_argument("BGM control module is invalid");
        }
        if (!std::isfinite(config.gainScale) ||
            config.gainScale < 0.25 || config.gainScale > 2.0) {
            throw std::invalid_argument(
                "BGM control gain scale must be finite and in [0.25, 2]");
        }
        if (!std::isfinite(config.rudderLimitDeg) ||
            config.rudderLimitDeg < 5.0 || config.rudderLimitDeg > 45.0) {
            throw std::invalid_argument(
                "BGM control rudder limit must be finite and in [5, 45] deg");
        }
        _config = config;
        reset();
    }

    void Control::reset() {
        alpha_cmd = 0.0;
        beta_cmd = 0.0;
        pre_ex = pre_ey = pre_ez = 0.0;
        ex = ey = ez = 0.0;
        iex = iey = iez = 0.0;
        last_dx = last_dy = last_dz = 0.0;
    }

    std::pair<Eigen::Vector3d, Eigen::Vector3d> Control::P6dof_Control(double step, const Eigen::Vector3d &acc_cmd_v, const State &state, const double totalMass,
                                                                       const Eigen::Vector3d &p_body, const ImuInfo &imu_info, const double rel_dis, const double rel_dis_dot,const double s) {
        auto lla                = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto velocity_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const auto theta = ModelDevelop::Utils::CoordinateHelper::getTheta(velocity_nue);

        // region 得标称弹道临时用
        constexpr double K1 = 4;
        const double Ma     = velocity_nue.norm() / 340.0;
        double ma=Ma;
        if (Ma < 0.1) {
            ma = 0.1;
        }
        double CN =0.3+ 0.6*ma*ma / (1 + 0.8*ma*ma*ma*ma) + 4.0 / sqrt(1 + (ma*ma - 1)*(ma*ma - 1));
        const auto rho = ModelDevelop::Utils::Aerodynamics::calculateAtmosphereDensity(lla.z());


        const double P         = p_body.norm();
        const double ka        = CN * 0.5 * rho * velocity_nue.squaredNorm() * s;
        const double m_a_alpha = acc_cmd_v.y() * totalMass;
        double alpha_c         = 0.0;  // 初始猜测
        const double kb        = -CN * 0.5 * rho * velocity_nue.squaredNorm() * s;
        const double m_a_beta  = acc_cmd_v.z() * totalMass;
        double beta_c          = 0.0;  // 初始猜测

        for (int i = 0; i < 5; i++) {                                       // 最多迭代20次
            const double f  = P * sin(alpha_c) + ka * alpha_c  - m_a_alpha; // 函数值
            const double df = P * cos(alpha_c) + ka;                        // 导数值
            alpha_c         = alpha_c - f / df;                             // 牛顿迭代
        }
        alpha_cmd          = alpha_c;

        for (int i = 0; i < 5; i++) {                                                   // 最多迭代20次
            const double f  = -P * cos(alpha_cmd)*sin(beta_c) + kb * beta_c  - m_a_beta; // 函数值
            const double df = -P * cos(alpha_cmd)*cos(beta_c) + kb;                       // 导数值
            beta_c          = beta_c - f / df;                                          // 牛顿迭代
        }
        beta_cmd          = beta_c;


        alpha_cmd = limit(alpha_cmd,-45/57.3,45/57.3);
        beta_cmd = limit(beta_cmd,-45/57.3,45/57.3);

        // endregion

        Eigen::Vector3d rudder;
        double alpha, beta;
        ModelDevelop::Utils::CoordinateHelper::calculateAngleOfAttack(velocity_nue, state.qbn, alpha, beta);
        const auto acc_cmd_body = ModelDevelop::Utils::CoordinateHelper::velocityToBodyAcceleration(acc_cmd_v, alpha, beta);
        const double wx         = imu_info.imu_w_xyz_body.x();
        const double wy         = imu_info.imu_w_xyz_body.y();
        const double wz         = imu_info.imu_w_xyz_body.z();

        if (rel_dis > rel_dis_dot * step * 10) {
            ex = 0 - imu_info.imu_ypr.z();
            ey = acc_cmd_body.z() - imu_info.imu_acc_body.z();
            ez = acc_cmd_body.y() - imu_info.imu_acc_body.y();
            if (_config.module == ControlModule::P6dofPi) {
                iex += (pre_ex + ex) * 0.5 * step;
                iey += (pre_ey + ey) * 0.5 * step;
                iez += (pre_ez + ez) * 0.5 * step;
            } else {
                iex = iey = iez = 0.0;
            }
            pre_ex = ex;
            pre_ey = ey;
            pre_ez = ez;

            rudder.z() = -alpha_cmd + 0.05 * wz + _config.gainScale * (-0.0001 * ez - 0.005 * iez);
            rudder.y() = -beta_cmd + 0.05 * wy + _config.gainScale * (+0.0001 * ey + 0.005 * iey);
            rudder.x() = 0.5 * wx + _config.gainScale * (-4.0 * ex - 0.1 * iex);

            const double rudderLimit = _config.rudderLimitDeg / 57.3;
            rudder.z() = limit(rudder.z(), -rudderLimit, rudderLimit);
            rudder.y() = limit(rudder.y(), -rudderLimit, rudderLimit);
            rudder.x() = limit(rudder.x(), -rudderLimit, rudderLimit);

            last_dx    = rudder.x();
            last_dy    = rudder.y();
            last_dz    = rudder.z();
        } else {
            rudder.z() = last_dz;
            rudder.y() = last_dy;
            rudder.x() = last_dx;
        }

        Eigen::Vector3d m_b;
        m_b.z() = 0;
        m_b.y() = 0;
        m_b.x() = 0;

        return {rudder, m_b};
    }

    double Control::firstOrderFilter(const double input, const double prev_output) {
        constexpr double Ts = 0.005;
        constexpr double T  = 0.005;
        return (Ts * input + T * prev_output) / (T + Ts);
    }

    double Control::limit(const double x, const double lower, const double upper) {
        return x < lower ? lower : (x > upper ? upper : x);
    }

// endregion

// region Get/Set选择器
// endregion

// region Private Methods
// endregion
}
#undef PRETTY_FILE_NAME
