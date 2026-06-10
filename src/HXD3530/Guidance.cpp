//
// Created by Administrator on 2026/1/29.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "HXD3530/Guidance.h"
#include "HXD3530/Seeker.h"
#include "CoordinateHelper.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/HXD/HXD"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::HXD3530 {
// region Static Attributes Init
    // endregion

    // region USING/FRIEND
    // endregion

    // region Constructor
    // endregion

    // region Public Methods
    GCInfo Guidance::getGCInfo(const double flyTime, const double P, const double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf,
                               const State &state, const double maxLoad) {
        const auto selfPositionEcf = state.posEcf;
        auto lla                   = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto target_dis      = (targetPosEcf - selfPositionEcf).norm();
        const auto selfVel_nue     = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const auto theta           = ModelDevelop::Utils::CoordinateHelper::getTheta(selfVel_nue);
        Eigen::Vector3d acc_cmd_v  = {0, 0, 0};
        LosInfo losInfo            = {};
        double g = Utils::CoordinateHelper::calculateGravity(state.posEcf).norm();
        if (flyTime < 2.6)// 策略 无控建立速度
        {
            acc_cmd_v.y() = 0;
            acc_cmd_v.z() = 0;
        }
        else if (target_dis > 300000) //策略 纵向巡航飞行，侧向朝向目标
        {
            // 制导指令
            losInfo             = getLOSInfo(targetPosEcf, targetVelEcf, state);
            acc_cmd_v           = guidance_pn(theta, losInfo.sigma_az_dot, losInfo.sigma_elv_dot, losInfo.dis_dot);
            constexpr double wn = 0.05;
            acc_cmd_v.y()       = g*cos(theta)+ wn*(35*1000-lla.z()) - 2*4*wn*selfVel_nue.y();
        }
        else //末制导
        {
            // 制导指令
            losInfo   = _seeker.getLOSInfo(targetPosEcf, targetVelEcf, state);
            acc_cmd_v = guidance_pn(theta, losInfo.sigma_az_dot, losInfo.sigma_elv_dot, losInfo.dis_dot);
        }
        if (acc_cmd_v.y() > 9.8 * maxLoad)
            acc_cmd_v.y() = 9.8 * maxLoad;
        if (acc_cmd_v.y() < -9.8 * maxLoad)
            acc_cmd_v.y() = -9.8 * maxLoad;
        if (acc_cmd_v.z() > 9.8 * maxLoad)
            acc_cmd_v.z() = 9.8 * maxLoad;
        if (acc_cmd_v.z() < -9.8 * maxLoad)
            acc_cmd_v.z() = -9.8 * maxLoad;
        GCInfo gc_info;
        gc_info.acc_cmd_v = acc_cmd_v;
        gc_info.losInfo   = losInfo;

        return gc_info;
    }

    // endregion

    // region Get/Set选择器
    // endregion

    // region Private Methods
    LosInfo Guidance::getLOSInfo(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state) {
        const Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);

        const auto target_position_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, lla.x(), lla.y());
        const auto position_nue        = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(state.posEcf, lla.x(), lla.y());
        const auto target_velocity_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(targetVelEcf, lla.x(), lla.y());
        const auto velocity_nue        = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());


        const auto rel_pos = (target_position_nue - position_nue).eval();
        const auto rel_vel = (target_velocity_nue - velocity_nue).eval();
        const auto rel_w   = (rel_pos.cross(rel_vel) / rel_pos.squaredNorm()).eval();
        const auto dis     = (target_position_nue - position_nue).norm();

        const auto theta = ModelDevelop::Utils::CoordinateHelper::getTheta(velocity_nue);
        const auto psi   = ModelDevelop::Utils::CoordinateHelper::getPsi(velocity_nue);

        const auto sigma_az_dot  = -rel_w.x() * sin(theta) * cos(psi) + rel_w.y() * cos(theta) + rel_w.z() * sin(theta) * sin(psi);
        const auto sigma_elv_dot = rel_w.x() * sin(psi) + rel_w.z() * cos(psi);

        const auto dis_dot = rel_pos.dot(rel_vel) / dis;

        LosInfo los_info{};
        los_info.dis_dot       = dis_dot;
        los_info.sigma_az_dot  = sigma_az_dot;
        los_info.sigma_elv_dot = sigma_elv_dot;
        los_info.sigma_elv     = ModelDevelop::Utils::CoordinateHelper::getTheta(rel_pos);
        los_info.sigma_az      = ModelDevelop::Utils::CoordinateHelper::getPsi(rel_pos);

        return los_info;
    }

    Eigen::Vector3d Guidance::guidance_pn(const double theta, const double sigma_az_dot, const double sigma_elv_dot, const double dis_dot) {
        constexpr double K     = 4;
        constexpr auto gravity = 9.8;
        const auto ny_tc       = K * fabs(dis_dot) * sigma_elv_dot + gravity * cos(theta);
        const auto nz_tc       = -K * fabs(dis_dot) * sigma_az_dot;
        auto acc_cmd_v         = Eigen::Vector3d(0, ny_tc, nz_tc);
        return acc_cmd_v;
    }

    void Guidance::Lambert_Resolve_Dv1(const Eigen::Vector3d &r_m, const Eigen::Vector3d &r_pip, double &T_pip, double vd_m[3], double &Range) const {
        double v_m[3];

        double gamma_min, gamma_max, gamma0;
        double V0, lambda, temp;
        Eigen::Vector3d i_vec, j_vec, Temp_Vec;

        double t0      = 0;
        double R_m     = r_m.norm();
        double R_pip   = r_pip.norm();
        double R_m_pip = r_m.dot(r_pip);
        double theta_f = acos(R_m_pip / R_m / R_pip);

        ////////////////////////////////////
        //theta_f = MyMisStatus.PI*2-Math.Acos(R_m_pip / R_m / R_pip);
        ////////////////////////////////////
        Range     = theta_f * earth_ae / 1000;
        double Ve = sqrt(2 * c_dMiu / R_m); //逃逸速度
        if (Range < 9000) {
            //弹道倾角范围：
            gamma_min = atan((cos(theta_f) - R_m / R_pip) / sin(theta_f));
            gamma_max = atan((sin(theta_f) + sqrt((1 - cos(theta_f)) * 2 * R_m / R_pip)) / (1 - cos(theta_f)));

            //迭代求解
            gamma0 = (gamma_min + gamma_max) / 2;
            V0     = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m / (R_m * (cos(gamma0) * cos(gamma0)) - R_pip * cos(theta_f + gamma0) * cos(gamma0));
            V0     = sqrt(V0);
            lambda = R_m * V0 * V0 / c_dMiu;

            if ((lambda > 0) && (lambda < 2)) {
                t0 = (tan(gamma0) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
                //temp = 1/Math.Tan(theta_f / 2);
                t0 = t0 + 2 * cos(gamma0) * atan(sqrt(2 / lambda - 1) / (cos(gamma0) *
                                                                         (1 / tan(theta_f / 2)) - sin(gamma0))) / lambda / pow((2 / lambda - 1), 1.5);
                t0 = t0 * R_m / V0 / cos(gamma0);
            }

            i_vec    = r_m / R_m;
            Temp_Vec = r_m.cross(r_pip);
            j_vec    = Temp_Vec.cross(r_m);
            temp     = j_vec.norm();
            j_vec    = j_vec / temp;

            vd_m[0] = V0 * sin(gamma0) * i_vec[0] + V0 * cos(gamma0) * j_vec[0];
            vd_m[1] = V0 * sin(gamma0) * i_vec[1] + V0 * cos(gamma0) * j_vec[1];
            vd_m[2] = V0 * sin(gamma0) * i_vec[2] + V0 * cos(gamma0) * j_vec[2];
            v_m[0]  = vd_m[0];
            v_m[1]  = vd_m[1];
            v_m[2]  = vd_m[2];
            T_pip   = t0;
        } else {
            T_pip = 2500 - (13000 - Range) / 7; //加入线性方程

            double mask, t_delt, kesi, gamma_d, Vd;
            int n;
            double gamma[1000], t_ff[1000], V[1000];

            kesi    = 0.001;
            mask    = 0;
            gamma_d = 0;
            Vd      = 0;
            //弹道倾角范围：
            gamma_min = atan((cos(theta_f) - R_m / R_pip) / sin(theta_f));
            gamma_max = atan((sin(theta_f) + sqrt((1 - cos(theta_f)) * 2 * R_m / R_pip)) / (1 - cos(theta_f)));

            //迭代求解
            gamma0 = (gamma_min + gamma_max) / 2;
            V0     = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m /
                     (R_m * (cos(gamma0) * cos(gamma0)) - R_pip * cos(theta_f + gamma0) * cos(gamma0));
            V0     = sqrt(V0);
            lambda = R_m * V0 * V0 / c_dMiu;

            if ((lambda > 0) && (lambda < 2)) {
                t0 = (tan(gamma0) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
                //temp = 1/Math.Tan(theta_f / 2);

                t0 = t0 + 2 * cos(gamma0) * atan(sqrt(2 / lambda - 1) / (cos(gamma0) *
                                                                         (1 / tan(theta_f / 2)) - sin(gamma0))) / lambda / pow((2 / lambda - 1), 1.5);
                t0 = t0 * R_m / V0 / cos(gamma0);
            }
            if (t0 > T_pip) {
                gamma[0] = (gamma_min + gamma0) / 2;
                mask     = 1;
            } else {
                gamma[0] = (gamma_max + gamma0) / 2;
                mask     = -1;
            }
            t_delt = T_pip;
            n      = 0;
            while (abs(t_delt) > kesi) {
                V[n] = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m /
                       (R_m * (cos(gamma[n]) * cos(gamma[n])) - R_pip * cos(theta_f + gamma[n]) * cos(gamma[n]));
                V[n]   = sqrt(V[n]);
                lambda = R_m * V[n] * V[n] / c_dMiu;
                if (lambda > 0 && lambda < 2) {
                    t_ff[n] = (tan(gamma[n]) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
                    t_ff[n] = t_ff[n] + 2 * cos(gamma[n]) * atan(sqrt(2 / lambda - 1) / (cos(gamma[n]) *
                                                                                         (1 / tan(theta_f / 2)) - sin(gamma[n]))) / lambda / pow((2 / lambda - 1), 1.5);
                    t_ff[n] = t_ff[n] * R_m / V[n] / cos(gamma[n]);
                } else {
                    t_ff[n] = t_ff[n - 1];
                }
                t_delt = T_pip - t_ff[n];
                if (abs(t_delt) > kesi) {
                    if (n == 0) {
                        gamma[n + 1] = gamma[n] + (gamma[n] - gamma0) * (T_pip - t_ff[n]) / (t_ff[n] - t0);
                    } else {
                        gamma[n + 1] = gamma[n] + (gamma[n] - gamma[n - 1]) * (T_pip - t_ff[n]) / (t_ff[n] - t_ff[n - 1]);
                    }
                    if ((gamma[n + 1] < gamma_min) || (gamma[n + 1] > gamma_max)) {
                        if (mask == 1) {
                            if (n == 0) {
                                gamma[n + 1] = (std::min(gamma[n], gamma0) + gamma_min) / 2;
                            } else {
                                gamma[n + 1] = (std::min(gamma[n], gamma[n - 1]) + gamma_min) / 2;
                            }
                        } else {
                            if (n == 0) {
                                gamma[n + 1] = (std::max(gamma[n], gamma0) + gamma_max) / 2;
                            } else {
                                gamma[n + 1] = (std::max(gamma[n], gamma[n - 1]) + gamma_max) / 2;
                            }
                        }
                    }
                } else {
                    gamma_d = gamma[n];
                    Vd      = V[n];
                }

                n = n + 1;
            }
            i_vec    = r_m / R_m;
            Temp_Vec = r_m.cross(r_pip);
            j_vec    = Temp_Vec.cross(r_m);
            temp     = j_vec.norm();
            j_vec    = j_vec / temp;

            vd_m[0] = Vd * sin(gamma_d) * i_vec[0] + Vd * cos(gamma_d) * j_vec[0];
            vd_m[1] = Vd * sin(gamma_d) * i_vec[1] + Vd * cos(gamma_d) * j_vec[1];
            vd_m[2] = Vd * sin(gamma_d) * i_vec[2] + Vd * cos(gamma_d) * j_vec[2];

            v_m[0] = vd_m[0];
            v_m[1] = vd_m[1];
            v_m[2] = vd_m[2];
        }
    }

    void Guidance::Lambert_Resolve_Dv(const Eigen::Vector3d &r_m, const Eigen::Vector3d &r_pip, double &T_pip, double vd_m[3], double &Range) const {
        double v_m[3];
        Eigen::Vector3d i_vec, j_vec, Temp_Vec;

        double t0      = 0;
        double R_m     = r_m.norm();
        double R_pip   = r_pip.norm();
        double R_m_pip = r_m.dot(r_pip);
        double theta_f = acos(R_m_pip / R_m / R_pip);

        ////////////////////////////////////
        //theta_f = MyMisStatus.PI*2-Math.Acos(R_m_pip / R_m / R_pip);
        ////////////////////////////////////
        Range     = theta_f * earth_ae / 1000;
        double Ve = sqrt(2 * c_dMiu / R_m); //逃逸速度

        double gamma[10000], t_ff[10000], V[10000];

        double kesi    = 0.0001;
        double mask    = 0;
        double gamma_d = 0;
        double Vd      = 0;
        //弹道倾角范围：
        double gamma_min = atan((cos(theta_f) - R_m / R_pip) / sin(theta_f));
        double gamma_max = atan((sin(theta_f) + sqrt((1 - cos(theta_f)) * 2 * R_m / R_pip)) / (1 - cos(theta_f)));

        //迭代求解
        double gamma0 = (gamma_min + gamma_max) / 2;
        double V0     = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m /
                        (R_m * (cos(gamma0) * cos(gamma0)) - R_pip * cos(theta_f + gamma0) * cos(gamma0));
        V0            = sqrt(V0);
        double lambda = R_m * V0 * V0 / c_dMiu;

        if ((lambda > 0) && (lambda < 2)) {
            t0 = (tan(gamma0) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
            //temp = 1/Math.Tan(theta_f / 2);

            t0 = t0 + 2 * cos(gamma0) * atan(sqrt(2 / lambda - 1) / (cos(gamma0) *
                                                                     (1 / tan(theta_f / 2)) - sin(gamma0))) / lambda / pow((2 / lambda - 1), 1.5);
            t0 = t0 * R_m / V0 / cos(gamma0);
        }
        if (t0 > T_pip) {
            gamma[0] = (gamma_min + gamma0) / 2;
            mask     = 1;
        } else {
            gamma[0] = (gamma_max + gamma0) / 2;
            mask     = -1;
        }
        double t_delt = T_pip;
        int n         = 0;
        while (abs(t_delt) > kesi) {
            V[n]   = R_pip * (1 - cos(theta_f)) * c_dMiu / R_m / (R_m * (cos(gamma[n]) * cos(gamma[n])) - R_pip * cos(theta_f + gamma[n]) * cos(gamma[n]));
            V[n]   = sqrt(V[n]);
            lambda = R_m * V[n] * V[n] / c_dMiu;
            if (lambda > 0 && lambda < 2) {
                t_ff[n] = (tan(gamma[n]) * (1 - cos(theta_f)) + (1 - lambda) * sin(theta_f)) / (2 - lambda) / R_m * R_pip;
                t_ff[n] = t_ff[n] + 2 * cos(gamma[n]) * atan(sqrt(2 / lambda - 1) / (cos(gamma[n]) *
                                                                                     (1 / tan(theta_f / 2)) - sin(gamma[n]))) / lambda / pow((2 / lambda - 1), 1.5);
                t_ff[n] = t_ff[n] * R_m / V[n] / cos(gamma[n]);
            } else {
                t_ff[n] = t_ff[n - 1];
                Vd      = 12e3;
                break;
            }

            t_delt = T_pip - t_ff[n];
            if (abs(t_delt) > kesi) {
                if (n == 0) {
                    gamma[n + 1] = gamma[n] + (gamma[n] - gamma0) * (T_pip - t_ff[n]) / (t_ff[n] - t0);
                } else {
                    gamma[n + 1] = gamma[n] + (gamma[n] - gamma[n - 1]) * (T_pip - t_ff[n]) / (t_ff[n] - t_ff[n - 1]);
                }
                if ((gamma[n + 1] < gamma_min) || (gamma[n + 1] > gamma_max)) {
                    if (mask == 1) {
                        if (n == 0) {
                            gamma[n + 1] = (std::min(gamma[n], gamma0) + gamma_min) / 2;
                        } else {
                            gamma[n + 1] = (std::min(gamma[n], gamma[n - 1]) + gamma_min) / 2;
                        }
                    } else {
                        if (n == 0) {
                            gamma[n + 1] = (std::max(gamma[n], gamma0) + gamma_max) / 2;
                        } else {
                            gamma[n + 1] = (std::max(gamma[n], gamma[n - 1]) + gamma_max) / 2;
                        }
                    }
                }
            } else {
                gamma_d = gamma[n];
                Vd      = V[n];
            }
            n = n + 1;
        }


        i_vec       = r_m / R_m;
        Temp_Vec    = r_m.cross(r_pip);
        j_vec       = Temp_Vec.cross(r_m);
        double temp = j_vec.norm();
        j_vec       = j_vec / temp;

        vd_m[0] = Vd * sin(gamma_d) * i_vec[0] + Vd * cos(gamma_d) * j_vec[0];
        vd_m[1] = Vd * sin(gamma_d) * i_vec[1] + Vd * cos(gamma_d) * j_vec[1];
        vd_m[2] = Vd * sin(gamma_d) * i_vec[2] + Vd * cos(gamma_d) * j_vec[2];

        v_m[0] = vd_m[0];
        v_m[1] = vd_m[1];
        v_m[2] = vd_m[2];
    }

// endregion
}
#undef PRETTY_FILE_NAME