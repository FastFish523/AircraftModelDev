//
// Created by 17298 on 2026/4/22.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "TGC/Guidance.h"
#include "TGC/Seeker.h"
#include "CoordinateHelper.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::TGC {
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
        if (flyTime < 2.6) // 策略 无控建立速度
        {
            acc_cmd_v.y() = 0;
            acc_cmd_v.z() = 0;
        } else if (P > 0 && target_dis > 15000) //策略 比例中制导
        {
            // 制导指令
            losInfo       = getLOSInfo(targetPosEcf, targetVelEcf, state);
            acc_cmd_v     = guidance_pn(theta, losInfo.sigma_az_dot, losInfo.sigma_elv_dot, losInfo.dis_dot);
            acc_cmd_v.y() = acc_cmd_v.y() + 2.5 * (losInfo.sigma_elv - (-20) / 57.3) * selfVel_nue.norm() / (target_dis / selfVel_nue.norm());
        } else //末制导
        {
            // 制导指令
            losInfo       = _seeker.getLOSInfo(targetPosEcf, targetVelEcf, state);
            acc_cmd_v     = guidance_pn(theta, losInfo.sigma_az_dot, losInfo.sigma_elv_dot, losInfo.dis_dot);
            acc_cmd_v.y() = acc_cmd_v.y() + 2.5 * (losInfo.sigma_elv - (-20) / 57.3) * selfVel_nue.norm() / (target_dis / selfVel_nue.norm());
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

    GCInfo Guidance::getGCInfoRouteL1(const State &state, const double maxLoad, const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex) {
        auto lla                  = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto selfVel_nue    = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double theta        = Utils::CoordinateHelper::getTheta(selfVel_nue);
        Eigen::Vector3d acc_cmd_v = {0, 0, 0};
        const double Vy           = selfVel_nue.y();

        auto [acc_cmd_vz,desired_h] = calculateL1Guidance(maxLoad, state.posEcf, state.velEcf, waypoints, currentWpIndex);
        lastDesiredH                = lastDesiredH + 0.001 * (desired_h - lastDesiredH);
        acc_cmd_v.z()               = acc_cmd_vz;
        acc_cmd_v.y()               = 9.8 * cos(theta) + 0.2 * (desired_h - lla.z()) - 2 * 4 * 0.2 * Vy;

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
        return gc_info;
    }

    std::pair<double, double> Guidance::calculateL1Guidance(const double maxLoad, const Eigen::Vector3d &currentPosEcf, const Eigen::Vector3d &currentVelEcf,
                                                            const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex) {
        // 检查输入
        if (waypoints.empty() || currentWpIndex < 0 ||
            currentWpIndex >= (int) waypoints.size() - 1) {
            currentWpIndex = -1;
            return {0.0, 0.0};
        }

        // 获取当前航段
        const auto &wp_start_lla     = waypoints[currentWpIndex];
        const auto &wp_end_lla       = waypoints[currentWpIndex + 1];
        Eigen::Vector3d wp_start_ecf = Utils::CoordinateHelper::llaToEcef(wp_start_lla);
        Eigen::Vector3d wp_end_ecf   = Utils::CoordinateHelper::llaToEcef(wp_end_lla);


        // 转换到NUE坐标系（以起点为原点）
        Eigen::Vector3d pos_nue = Utils::CoordinateHelper::ecefToNuePosition(currentPosEcf, wp_start_lla.x(), wp_start_lla.y()) - Utils::CoordinateHelper::ecefToNuePosition(
                                      wp_start_ecf, wp_start_lla.x(), wp_start_lla.y());

        Eigen::Vector3d wp_start_nue(0, 0, 0); // 起点是原点

        Eigen::Vector3d wp_end_nue = Utils::CoordinateHelper::ecefToNuePosition(wp_end_ecf, wp_start_lla.x(), wp_start_lla.y()) - Utils::CoordinateHelper::ecefToNuePosition(
                                         wp_start_ecf, wp_start_lla.x(), wp_start_lla.y());

        // 速度也需要转换到NUE
        Eigen::Vector3d vel_nue3 = Utils::CoordinateHelper::ecefToNueVelocity(currentVelEcf, wp_start_lla.x(), wp_start_lla.y());
        Eigen::Vector2d vel_nue(vel_nue3.x(), vel_nue3.z());

        // 计算最小转弯半径
        double speed = vel_nue.norm();
        double R_min = (speed * speed) / (maxLoad * 9.8);
        // L1应该大于最小转弯半径
        double L1_distance = 3 * R_min; // 经验系数

        // 只使用北-东平面
        Eigen::Vector2d pos_2d(pos_nue[0], pos_nue[2]);
        Eigen::Vector2d wp_start_2d(wp_start_nue[0], wp_start_nue[2]);
        Eigen::Vector2d wp_end_2d(wp_end_nue[0], wp_end_nue[2]);

        // 计算横向加速度
        double acc = calculateL1GuidanceNUE(
            pos_2d, vel_nue, wp_start_2d, wp_end_2d, L1_distance);

        // 检查是否应该切换到下一个航点
        if (currentWpIndex <= (int) waypoints.size() - 2) {
            // 计算到终点的距离
            Eigen::Vector2d to_end = wp_end_2d - pos_2d;
            double dist_to_end     = to_end.norm();
            // 如果接近当前航段终点，切换到下一个航段
            if (dist_to_end < L1_distance * 0.01) {
                currentWpIndex++;
                std::cout << "next" << std::endl;
            }
        }

        return {acc, wp_end_lla.z()};
    }

    double Guidance::calculateL1GuidanceNUE(const Eigen::Vector2d &pos_nue, const Eigen::Vector2d &vel_nue, const Eigen::Vector2d &wp_start, const Eigen::Vector2d &wp_end,
                                            double L1_distance) {
        // 1. 计算航段向量
        Eigen::Vector2d segment = wp_end - wp_start;
        double seg_length       = segment.norm();

        if (seg_length < 1e-6) {
            return 0.0;
        }

        // 2. 计算航段单位向量
        Eigen::Vector2d seg_unit = segment / seg_length;

        // 3. 计算侧向误差向量
        Eigen::Vector2d rel_pos = pos_nue - wp_start; // 相对起点位置

        // 在航段上的投影长度
        double s = rel_pos.dot(seg_unit);

        // 投影点坐标
        Eigen::Vector2d proj_point = wp_start + seg_unit * s;

        // 侧向误差向量（从投影点到当前位置）
        Eigen::Vector2d lateral_vec = pos_nue - proj_point;

        // 4. 计算L1点
        // L1点在航段上，距离投影点L1_distance
        double s_L1 = s + L1_distance;

        Eigen::Vector2d L1_point;
        if (s_L1 > seg_length) {
            L1_point = wp_end; // 如果超出终点，用终点
        } else {
            L1_point = wp_start + seg_unit * s_L1;
        }

        // 5. 计算到L1点的向量
        Eigen::Vector2d vec_to_L1 = L1_point - pos_nue;
        double dist_to_L1         = vec_to_L1.norm();

        if (dist_to_L1 < 1e-6) {
            return 0.0;
        }

        // 6. 计算速度与到L1点向量的夹角
        double V = vel_nue.norm();
        if (V < 1e-6) {
            return 0.0;
        }

        // 计算sin(η)，η是速度与vec_to_L1的夹角
        // 在NUE坐标系，叉积公式：a × b = a_x*b_z - a_z*b_x（忽略Y轴）
        // 返回的Y分量表示旋转方向
        double cross_y = vel_nue[0] * vec_to_L1[1] - vel_nue[1] * vec_to_L1[0];
        double sin_eta = cross_y / (V * dist_to_L1);

        // 7. 计算L1制导加速度
        // 公式: a_cmd = 2 * V^2 * sin(η) / L
        double lateral_acc = 2.0 * V * V * sin_eta / dist_to_L1;

        return lateral_acc;
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