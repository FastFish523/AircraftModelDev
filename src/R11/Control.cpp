//
// Created by Administrator on 2026/2/2.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "R11/Control.h"
#include "Aerodynamics.h"
#include "CoordinateHelper.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/R11/R11"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::R11 {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
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

        double alpha_c         = 0.0;  // 初始猜测
        double beta_c          = 0.0;
        alpha_cmd          = alpha_c;
        beta_cmd          = beta_c;

        alpha_cmd = limit(alpha_cmd,-45/57.3,45/57.3);
        beta_cmd = limit(beta_cmd,-45/57.3,45/57.3);

        // endregion

        Eigen::Vector3d rudder{0,0,0};
        Eigen::Vector3d m_b{0,0,0};
        double alpha, beta;
        ModelDevelop::Utils::CoordinateHelper::calculateAngleOfAttack(velocity_nue, state.qbn, alpha, beta);
        const auto acc_cmd_body = ModelDevelop::Utils::CoordinateHelper::velocityToBodyAcceleration(acc_cmd_v, alpha, beta);
        const double wx         = imu_info.imu_w_xyz_body.x();
        const double wy         = imu_info.imu_w_xyz_body.y();
        const double wz         = imu_info.imu_w_xyz_body.z();

        if (rel_dis > rel_dis_dot * step * 10) {
            ex = 0 - imu_info.imu_ypr.z();
            ey = beta_c - beta;
            ez = alpha_c - alpha;
            iex += (pre_ex + ex) * 0.5 * step;
            iey += (pre_ey + ey) * 0.5 * step;
            iez += (pre_ez + ez) * 0.5 * step;
            pre_ex = ex;
            pre_ey = ey;
            pre_ez = ez;

            m_b.z() = -3000 * wz +  1 * (5000 * ez + 0 * iez);
            m_b.y() = -3000 * wy + 1  * (5000 * ey + 0 * iey);
            m_b.x() = -3000 * wx + 5000 * ex - 0 * iex;

            last_dx    = m_b.x();
            last_dy    = m_b.y();
            last_dz    = m_b.z();
        } else {
            m_b.z() = last_dz;
            m_b.y() = last_dy;
            m_b.x() = last_dx;
        }



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