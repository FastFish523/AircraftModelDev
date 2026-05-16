//
// Created by 17298 on 2026/4/22.
//

// region Include
// region STL
#include <algorithm>
#include <cmath>
// endregion
// region ThirdParty
// endregion
// region Self
#include "TGC/Guidance.h"
#include "TGC/Engine.h"
#include "TGC/Seeker.h"
#include "TGC/BoostConfig.h"
#include "CoordinateHelper.h"
#include "Constants.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    GCInfo Guidance::getMissionGCInfo(const double flyTime, const double P, const double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf,
                                      const State &state, const double maxLoad, const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex) {
        constexpr double safeSeparationTime = 0.4;
        constexpr double seekerAcquireDistance = 20000.0;
        constexpr double handoverEndDistance = 12000.0;
        constexpr double seekerHalfFov = 60.0 / 57.3;

        GCInfo gc_info;
        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto selfVel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double theta = ModelDevelop::Utils::CoordinateHelper::getTheta(selfVel_nue);
        const double psi = ModelDevelop::Utils::CoordinateHelper::getPsi(selfVel_nue);
        const double target_dis = (targetPosEcf - state.posEcf).norm();
        const bool hasRoute = !waypoints.empty() && currentWpIndex >= 0 && currentWpIndex < static_cast<int>(waypoints.size()) - 1;

        const auto los_midcourse = getLOSInfo(targetPosEcf, targetVelEcf, state);
        const auto los_terminal = _seeker.getLOSInfo(targetPosEcf, targetVelEcf, state);
        const bool seekerInFov = std::abs(los_terminal.sigma_az_b) <= seekerHalfFov && std::abs(los_terminal.sigma_elv_b) <= seekerHalfFov;
        //const bool seekerLocked = target_dis <= seekerAcquireDistance && seekerInFov;
        const bool seekerLocked = target_dis <= seekerAcquireDistance;

        Eigen::Vector3d acc_cmd_v = Eigen::Vector3d::Zero();
        double handoverRatio = 0.0;

        if (flyTime < safeSeparationTime) {
            gc_info.phase = GuidancePhase::Boost;
            gc_info.losInfo = los_midcourse;
        } else if (P > 1.0e3) {
            gc_info.phase = GuidancePhase::Boost;
            gc_info.losInfo = los_midcourse;
            const auto boostInfo = calculateBoostGuidance(flyTime, P, Mass, targetPosEcf, maxLoad, state);
            acc_cmd_v = boostInfo.acc_cmd_v;
            gc_info.tvc_cmd = boostInfo.tvc_cmd;
            gc_info.pitch_cmd = boostInfo.pitch_cmd;
            gc_info.pitch_cmd_valid = boostInfo.pitch_cmd_valid;
        } else if (hasRoute && !seekerLocked) {
            gc_info = getGCInfoRouteL1(state, maxLoad, waypoints, currentWpIndex);
            gc_info.phase = GuidancePhase::Glide;
            gc_info.losInfo = los_midcourse;
            return gc_info;
        } else {
            const auto terminal_acc = guidance_pn(theta, los_terminal.sigma_az_dot, los_terminal.sigma_elv_dot, los_terminal.dis_dot);
            gc_info.losInfo = los_terminal;

            if (hasRoute && seekerLocked && target_dis > handoverEndDistance) {
                auto routeInfo = getGCInfoRouteL1(state, maxLoad, waypoints, currentWpIndex);
                handoverRatio = computeBlendRatio(target_dis, seekerAcquireDistance, handoverEndDistance);
                acc_cmd_v = (1.0 - handoverRatio) * routeInfo.acc_cmd_v + handoverRatio * terminal_acc;
                gc_info.phase = GuidancePhase::Handover;
            } else {
                acc_cmd_v = terminal_acc;
                gc_info.phase = GuidancePhase::Terminal;
            }
        }

        acc_cmd_v.y() = clamp(acc_cmd_v.y(), -9.8 * maxLoad, 9.8 * maxLoad);
        acc_cmd_v.z() = clamp(acc_cmd_v.z(), -9.8 * maxLoad, 9.8 * maxLoad);
        gc_info.acc_cmd_v = acc_cmd_v;
        gc_info.handoverRatio = handoverRatio;
        return gc_info;
    }

    Guidance::BoostGuidanceInfo Guidance::calculateBoostGuidance(const double flyTime, const double P, const double Mass,
                                                                 const Eigen::Vector3d &targetPosEcf, const double maxLoad,
                                                                 const State &state) {
        using namespace BoostConfig;
        constexpr double degToRad = Utils::Constants::DEG_TO_RAD;

        const auto burnOutTimes = Engine::boostBurnOutTimes();
        const double firstStageEnd = burnOutTimes[0];
        const double secondStageEnd = burnOutTimes[1];
        const double thirdStageEnd = burnOutTimes[2];

        BoostGuidanceInfo boostInfo;
        if (flyTime >= thirdStageEnd || P <= 1.0e3 || Mass <= 1.0) {
            return boostInfo;
        }

        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto velocityNue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double speed = velocityNue.norm();
        if (speed < 1.0e-6) {
            return boostInfo;
        }

        const double theta = ModelDevelop::Utils::CoordinateHelper::getTheta(velocityNue);
        const double psi = ModelDevelop::Utils::CoordinateHelper::getPsi(velocityNue);
        if (!boostInitialPsi.has_value()) {
            boostInitialPsi = psi;
        }

        const auto losToTarget = getLOSInfo(targetPosEcf, Eigen::Vector3d::Zero(), state);
        const double targetPsi = losToTarget.sigma_az;
        double thetaCmd = STAGE1_START_BLEND_THETA * degToRad;
        double psiCmd = boostInitialPsi.value();
        double tvcLimit = TVC_LIMIT_STAGE1 * degToRad;

        if (flyTime <= firstStageEnd) {
            const double ratio = smoothStep(flyTime / firstStageEnd);
            thetaCmd = (STAGE1_START_BLEND_THETA + (STAGE1_END_TARGET_THETA - STAGE1_START_BLEND_THETA) * ratio) * degToRad;
        } else if (flyTime <= secondStageEnd) {
            const double ratio = smoothStep((flyTime - firstStageEnd) / (secondStageEnd - firstStageEnd));
            thetaCmd = (STAGE2_START_BLEND_THETA + (STAGE2_END_TARGET_THETA - STAGE2_START_BLEND_THETA) * ratio) * degToRad;
            tvcLimit = TVC_LIMIT_STAGE2 * degToRad;
        } else if (flyTime <= THIRD_STAGE_SPLIT_TIME) {
            const double ratio = smoothStep((flyTime - secondStageEnd) / (THIRD_STAGE_SPLIT_TIME - secondStageEnd));
            thetaCmd = (STAGE3_SPLIT_START_THETA + (STAGE3_SPLIT_END_THETA - STAGE3_SPLIT_START_THETA) * ratio) * degToRad;
            psiCmd = boostInitialPsi.value() + wrapAngle(targetPsi - boostInitialPsi.value()) * ratio;
            tvcLimit = TVC_LIMIT_STAGE3_SPLIT * degToRad;
        } else {
            const double ratio = smoothStep((flyTime - THIRD_STAGE_SPLIT_TIME) / (thirdStageEnd - THIRD_STAGE_SPLIT_TIME));
            const double altitudeError = TARGET_BURNOUT_ALTITUDE - lla.z();
            const double velocityError = TARGET_BURNOUT_VELOCITY - speed;
            const double terminalCorrection = clamp(altitudeError * TERMINAL_ALTITUDE_GAIN + velocityError * TERMINAL_VELOCITY_GAIN,
                                                    -TERMINAL_CORRECTION_LIMIT * degToRad, TERMINAL_CORRECTION_LIMIT * degToRad);
            thetaCmd = (STAGE3_FINAL_START_THETA + (STAGE3_FINAL_END_THETA - STAGE3_FINAL_START_THETA) * ratio) * degToRad + terminalCorrection;
            psiCmd = targetPsi;
            tvcLimit = TVC_LIMIT_STAGE3_FINAL * degToRad;
        }

        const double thetaError = wrapAngle(thetaCmd - theta);
        const double psiError = wrapAngle(psiCmd - psi);
        Eigen::Vector3d accCmdV = Eigen::Vector3d::Zero();
        accCmdV.y() = 9.8 * std::cos(theta) + BOOST_SPEED_THETA_GAIN * speed * thetaError - BOOST_DAMPING_GAIN * speed * std::sin(thetaError);
        accCmdV.z() = -BOOST_PSI_GAIN * speed * psiError;
        accCmdV.y() = clamp(accCmdV.y(), -9.8 * maxLoad, 9.8 * maxLoad);
        accCmdV.z() = clamp(accCmdV.z(), -9.8 * maxLoad, 9.8 * maxLoad);

        double alpha = 0.0;
        double beta = 0.0;
        ModelDevelop::Utils::CoordinateHelper::calculateAngleOfAttack(velocityNue, state.qbn, alpha, beta);
        const auto accCmdBody = ModelDevelop::Utils::CoordinateHelper::velocityToBodyAcceleration(accCmdV, alpha, beta);
        Eigen::Vector3d tvcCmd = Eigen::Vector3d::Zero();
        tvcCmd.x() = 0.0;
        tvcCmd.y() = clamp(2.0 * Mass * accCmdBody.y() / P, -tvcLimit, tvcLimit);
        tvcCmd.z() = clamp(2.0 * Mass * accCmdBody.z() / P, -tvcLimit, tvcLimit);

        boostInfo.acc_cmd_v = accCmdV;
        boostInfo.tvc_cmd = tvcCmd;
        boostInfo.pitch_cmd = thetaCmd + 15.0 * degToRad;
        boostInfo.pitch_cmd_valid = true;
        return boostInfo;
    }

    LosInfo Guidance::getLOSInfo(const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state) {
        const Eigen::Vector3d lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);

        const auto target_position_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, lla.x(), lla.y());
        const auto position_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(state.posEcf, lla.x(), lla.y());
        const auto target_velocity_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(targetVelEcf, lla.x(), lla.y());
        const auto velocity_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());

        const auto rel_pos = (target_position_nue - position_nue).eval();
        const auto rel_vel = (target_velocity_nue - velocity_nue).eval();
        const auto rel_w = (rel_pos.cross(rel_vel) / rel_pos.squaredNorm()).eval();
        const auto dis = (target_position_nue - position_nue).norm();

        const auto theta = ModelDevelop::Utils::CoordinateHelper::getTheta(velocity_nue);
        const auto psi = ModelDevelop::Utils::CoordinateHelper::getPsi(velocity_nue);

        const auto sigma_az_dot = -rel_w.x() * std::sin(theta) * std::cos(psi) + rel_w.y() * std::cos(theta) + rel_w.z() * std::sin(theta) * std::sin(psi);
        const auto sigma_elv_dot = rel_w.x() * std::sin(psi) + rel_w.z() * std::cos(psi);
        const auto dis_dot = rel_pos.dot(rel_vel) / dis;
        const auto rel_pos_body = ModelDevelop::Utils::CoordinateHelper::nueToBodyVector(rel_pos, state.qbn);

        LosInfo los_info{};
        los_info.dis_dot = dis_dot;
        los_info.sigma_az_dot = sigma_az_dot;
        los_info.sigma_elv_dot = sigma_elv_dot;
        los_info.sigma_elv = ModelDevelop::Utils::CoordinateHelper::getTheta(rel_pos);
        los_info.sigma_az = ModelDevelop::Utils::CoordinateHelper::getPsi(rel_pos);
        los_info.sigma_elv_b = std::atan2(rel_pos_body.y(), rel_pos_body.x());
        los_info.sigma_az_b = std::atan2(-rel_pos_body.z(), rel_pos_body.x());
        return los_info;
    }

    GCInfo Guidance::getGCInfoRouteL1(const State &state, const double maxLoad, const std::deque<Eigen::Vector3d> &waypoints, int &currentWpIndex) {
        auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto selfVel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double theta = Utils::CoordinateHelper::getTheta(selfVel_nue);
        Eigen::Vector3d acc_cmd_v = {0, 0, 0};
        const double Vy = selfVel_nue.y();

        auto [acc_cmd_vz, desired_h] = calculateL1Guidance(maxLoad, state.posEcf, state.velEcf, waypoints, currentWpIndex);
        lastDesiredH = lastDesiredH + 0.001 * (desired_h - lastDesiredH);
        acc_cmd_v.z() = acc_cmd_vz;
        acc_cmd_v.y() = 9.8 * std::cos(theta) + 0.2 * (desired_h - lla.z()) - 2 * 4 * 0.2 * Vy;

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
        if (waypoints.empty() || currentWpIndex < 0 || currentWpIndex >= static_cast<int>(waypoints.size()) - 1) {
            currentWpIndex = -1;
            return {0.0, 0.0};
        }

        const auto &wp_start_lla = waypoints[currentWpIndex];
        const auto &wp_end_lla = waypoints[currentWpIndex + 1];
        Eigen::Vector3d wp_start_ecf = Utils::CoordinateHelper::llaToEcef(wp_start_lla);
        Eigen::Vector3d wp_end_ecf = Utils::CoordinateHelper::llaToEcef(wp_end_lla);

        Eigen::Vector3d pos_nue = Utils::CoordinateHelper::ecefToNuePosition(currentPosEcf, wp_start_lla.x(), wp_start_lla.y()) -
                                  Utils::CoordinateHelper::ecefToNuePosition(wp_start_ecf, wp_start_lla.x(), wp_start_lla.y());
        Eigen::Vector3d wp_start_nue(0, 0, 0);
        Eigen::Vector3d wp_end_nue = Utils::CoordinateHelper::ecefToNuePosition(wp_end_ecf, wp_start_lla.x(), wp_start_lla.y()) -
                                     Utils::CoordinateHelper::ecefToNuePosition(wp_start_ecf, wp_start_lla.x(), wp_start_lla.y());

        Eigen::Vector3d vel_nue3 = Utils::CoordinateHelper::ecefToNueVelocity(currentVelEcf, wp_start_lla.x(), wp_start_lla.y());
        Eigen::Vector2d vel_nue(vel_nue3.x(), vel_nue3.z());

        double speed = vel_nue.norm();
        double R_min = (speed * speed) / (maxLoad * 9.8);
        double L1_distance = std::max(3.0 * R_min, 1000.0);

        Eigen::Vector2d pos_2d(pos_nue[0], pos_nue[2]);
        Eigen::Vector2d wp_start_2d(wp_start_nue[0], wp_start_nue[2]);
        Eigen::Vector2d wp_end_2d(wp_end_nue[0], wp_end_nue[2]);

        double acc = calculateL1GuidanceNUE(pos_2d, vel_nue, wp_start_2d, wp_end_2d, L1_distance);

        if (currentWpIndex <= static_cast<int>(waypoints.size()) - 2) {
            Eigen::Vector2d to_end = wp_end_2d - pos_2d;
            double dist_to_end = to_end.norm();
            if (dist_to_end < std::max(0.5 * L1_distance, 1000.0)) {
                currentWpIndex++;
                if (currentWpIndex >= static_cast<int>(waypoints.size()) - 1) {
                    currentWpIndex = -1;
                }
            }
        }

        return {acc, wp_end_lla.z()};
    }

    double Guidance::calculateL1GuidanceNUE(const Eigen::Vector2d &pos_nue, const Eigen::Vector2d &vel_nue, const Eigen::Vector2d &wp_start, const Eigen::Vector2d &wp_end,
                                            double L1_distance) {
        Eigen::Vector2d segment = wp_end - wp_start;
        double seg_length = segment.norm();
        if (seg_length < 1e-6) {
            return 0.0;
        }

        Eigen::Vector2d seg_unit = segment / seg_length;
        Eigen::Vector2d rel_pos = pos_nue - wp_start;
        double s = rel_pos.dot(seg_unit);
        Eigen::Vector2d proj_point = wp_start + seg_unit * s;
        double s_L1 = s + L1_distance;

        Eigen::Vector2d L1_point = s_L1 > seg_length ? wp_end : wp_start + seg_unit * s_L1;
        Eigen::Vector2d vec_to_L1 = L1_point - pos_nue;
        double dist_to_L1 = vec_to_L1.norm();
        if (dist_to_L1 < 1e-6) {
            return 0.0;
        }

        double V = vel_nue.norm();
        if (V < 1e-6) {
            return 0.0;
        }

        double cross_y = vel_nue[0] * vec_to_L1[1] - vel_nue[1] * vec_to_L1[0];
        double sin_eta = cross_y / (V * dist_to_L1);
        return 2.0 * V * V * sin_eta / dist_to_L1;
    }

    Eigen::Vector3d Guidance::guidance_pn(const double theta, const double sigma_az_dot, const double sigma_elv_dot, const double dis_dot) {
        constexpr double K = 4;
        constexpr double g = 9.8;
        const auto ny_tc = K * std::fabs(dis_dot) * sigma_elv_dot + g * std::cos(theta);
        const auto nz_tc = -K * std::fabs(dis_dot) * sigma_az_dot;
        return Eigen::Vector3d(0, ny_tc, nz_tc);
    }

    double Guidance::wrapAngle(double angle) {
        constexpr double pi = Utils::Constants::PI;
        constexpr double twoPi = 2.0 * pi;
        while (angle > pi) {
            angle -= twoPi;
        }
        while (angle < -pi) {
            angle += twoPi;
        }
        return angle;
    }

    double Guidance::clamp(const double value, const double minValue, const double maxValue) {
        return std::max(minValue, std::min(value, maxValue));
    }

    double Guidance::computeBlendRatio(const double targetDistance, const double startDistance, const double endDistance) {
        if (startDistance <= endDistance) {
            return 1.0;
        }
        return clamp((startDistance - targetDistance) / (startDistance - endDistance), 0.0, 1.0);
    }

    double Guidance::smoothStep(const double ratio) {
        const double x = clamp(ratio, 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    void Guidance::reset() {
        lastDesiredH = 0.0;
        boostInitialPsi = std::nullopt;
    }

}

#undef PRETTY_FILE_NAME
