//
// Created by 17298 on 2026/4/22.
//

// region Include
// region STL
#include <algorithm>
#include <cmath>
#include <limits>
// endregion
// region ThirdParty
// endregion
// region Self
#include "TGC/Guidance.h"
#include "TGC/Engine.h"
#include "TGC/Seeker.h"
#include "TGC/BoostConfig.h"
#include "Aerodynamics.h"
#include "CoordinateHelper.h"
#include "Constants.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TGC/TGC"
// endregion

namespace ModelDevelop::TGC {
    GCInfo Guidance::getMissionGCInfo(const double flyTime, const double P, const double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf,
                                      const State &state, const double maxLoad) {
        constexpr double safeSeparationTime = 0.4;
        constexpr double postBoostClimbTime = 20.0;
        constexpr double seekerAcquireDistance = 260000.0;
        constexpr double handoverEndDistance = 260000.0;
        constexpr double climbThetaCmd = 18.0 / 57.3;

        GCInfo gc_info;
        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto selfVel_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double theta = ModelDevelop::Utils::CoordinateHelper::getTheta(selfVel_nue);
        const double psi = ModelDevelop::Utils::CoordinateHelper::getPsi(selfVel_nue);
        gc_info.theta_cmd = theta;
        const double target_dis = (targetPosEcf - state.posEcf).norm();
        const DiveGuidanceConfig diveConfig{};
        const bool inDiveEnvelope = target_dis <= diveConfig.entryDistance;

        const auto los_midcourse = getLOSInfo(targetPosEcf, targetVelEcf, state);
        const auto los_terminal = getLOSInfo(targetPosEcf, targetVelEcf, state);
        const bool seekerLocked = target_dis <= seekerAcquireDistance;

        Eigen::Vector3d acc_cmd_v = Eigen::Vector3d::Zero();
        double handoverRatio = 0.0;

        if (flyTime < safeSeparationTime) {
            gc_info.phase = GuidancePhase::Boost;
            gc_info.losInfo = los_midcourse;
            gc_info.theta_cmd = BoostConfig::LAUNCH_THETA_F / 57.3;
        } else if (P > 1.0e3 && flyTime < Engine::boostTotalTime()) {
            gc_info.phase = GuidancePhase::Boost;
            gc_info.losInfo = los_midcourse;
            const auto boostInfo = calculateBoostGuidance(flyTime, P, Mass, targetPosEcf, maxLoad, state);
            acc_cmd_v = boostInfo.acc_cmd_v;
            gc_info.tvc_cmd = boostInfo.tvc_cmd;
            gc_info.theta_cmd = boostInfo.theta_cmd;
            gc_info.pitch_cmd = boostInfo.pitch_cmd;
            gc_info.pitch_cmd_valid = boostInfo.pitch_cmd_valid;
        } else if (flyTime < Engine::boostTotalTime() + postBoostClimbTime) {
            gc_info.phase = GuidancePhase::Climb;
            gc_info.losInfo = los_midcourse;

            const double heading_error = wrapAngle(los_midcourse.sigma_az - psi);
            const double lateral_acc = 2.0 * selfVel_nue.norm() * selfVel_nue.norm() * std::sin(heading_error) / std::max(target_dis, 1000.0);

            const double theta_error = climbThetaCmd - theta;
            const double speed = selfVel_nue.norm();
            const double vy_cmd = speed * std::sin(climbThetaCmd);
            acc_cmd_v.y() = 9.8 * std::cos(theta)  + 1.8 * speed * theta_error - 0.45 * (selfVel_nue.y() - vy_cmd);
            acc_cmd_v.z() = lateral_acc;
            gc_info.theta_cmd = climbThetaCmd;
        } else if (!seekerLocked) {
            gc_info = getGCInfoAnalyticMidcourse(flyTime, targetPosEcf, targetVelEcf, state, Mass, maxLoad);
            return gc_info;
        } else if (inDiveEnvelope) {
            gc_info = getDiveGCInfo(flyTime, targetPosEcf, targetVelEcf, state, maxLoad, diveConfig);
            return gc_info;
        } /*else {
            const auto terminal_acc = guidance_pn(theta, los_terminal.sigma_az_dot, los_terminal.sigma_elv_dot, los_terminal.dis_dot);
            gc_info.losInfo = los_terminal;

            if (seekerLocked && target_dis > handoverEndDistance) {
                auto routeInfo = getGCInfoAnalyticMidcourse(flyTime, targetPosEcf, targetVelEcf, state, Mass, maxLoad);
                handoverRatio = computeBlendRatio(target_dis, seekerAcquireDistance, handoverEndDistance);
                acc_cmd_v = (1.0 - handoverRatio) * routeInfo.acc_cmd_v + handoverRatio * terminal_acc;
                gc_info.phase = GuidancePhase::Handover;
            } else {
                acc_cmd_v = terminal_acc;
                gc_info.phase = GuidancePhase::Terminal;
            }
        }*/

        /*acc_cmd_v.y() = clamp(acc_cmd_v.y(), -9.8 * maxLoad, 9.8 * maxLoad);
        acc_cmd_v.z() = clamp(acc_cmd_v.z(), -9.8 * maxLoad, 9.8 * maxLoad);
        gc_info.acc_cmd_v = acc_cmd_v;
        gc_info.handoverRatio = handoverRatio;
        return gc_info;*/
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
        boostInfo.theta_cmd = thetaCmd;
        boostInfo.pitch_cmd = thetaCmd + 15.0 * degToRad;
        boostInfo.pitch_cmd_valid = true;
        return boostInfo;
    }

    GCInfo Guidance::getDiveGCInfo(const double flyTime, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, const double maxLoad,
                                   const DiveGuidanceConfig &config) {
        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto targetLla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(targetPosEcf);
        const auto selfVelNue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const auto relPosNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, lla.x(), lla.y()) -
                               ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(state.posEcf, lla.x(), lla.y());

        const double speed = selfVelNue.norm();
        const double theta = ModelDevelop::Utils::CoordinateHelper::getTheta(selfVelNue);
        const double psi = ModelDevelop::Utils::CoordinateHelper::getPsi(selfVelNue);
        const double targetDis = (targetPosEcf - state.posEcf).norm();
        const double horizontalDis = std::hypot(relPosNue.x(), relPosNue.z());

        const auto losMidcourse = getLOSInfo(targetPosEcf, targetVelEcf, state);
        const auto losTerminal = getLOSInfo(targetPosEcf, targetVelEcf, state);
        const double closingVelocity = -losTerminal.dis_dot;
        const bool targetClosing = closingVelocity > config.minClosingVelocity;
        const double timeToGo = targetClosing ? targetDis / closingVelocity : std::numeric_limits<double>::infinity();
        const bool seekerLocked = targetClosing && (targetDis <= config.handoverDistance || timeToGo <= config.terminalTime);

        double thetaCmdLimit = config.entryThetaCmd;
        GuidancePhase phase = GuidancePhase::DiveEntry;
        const bool terminalReady = seekerLocked && (targetDis <= config.terminalDistance || timeToGo <= config.terminalTime);
        const bool handoverReady = seekerLocked && targetDis <= config.handoverDistance;
        if (terminalReady) {
            thetaCmdLimit = config.terminalThetaCmd;
            phase = GuidancePhase::DiveTerminal;
        } else if (handoverReady) {
            thetaCmdLimit = config.midThetaCmd;
            phase = GuidancePhase::DiveHandover;
        } else if (targetDis <= config.midDistance) {
            thetaCmdLimit = config.midThetaCmd;
            phase = GuidancePhase::DiveMid;
        }
        if (_lastDivePhase.has_value() && static_cast<int>(phase) < static_cast<int>(*_lastDivePhase)) {
            phase = *_lastDivePhase;
        } else {
            _lastDivePhase = phase;
        }
        if (phase == GuidancePhase::DiveTerminal) {
            thetaCmdLimit = config.terminalThetaCmd;
        } else if (phase == GuidancePhase::DiveMid || phase == GuidancePhase::DiveHandover) {
            thetaCmdLimit = config.midThetaCmd;
        } else {
            thetaCmdLimit = config.entryThetaCmd;
        }

        constexpr double shallowDiveLimit = -5.0 / 57.3;
        const double heightToGo = lla.z() - targetLla.z();
        const double geometricThetaCmd = -std::atan2(std::max(0.0, heightToGo), std::max(horizontalDis, 1.0));
        const double rawThetaCmd = clamp(geometricThetaCmd, thetaCmdLimit, shallowDiveLimit);
        double thetaCmd = rawThetaCmd;
        if (_lastDiveThetaCmd.has_value() && _lastDiveThetaCmdTime.has_value() && flyTime > *_lastDiveThetaCmdTime) {
            const double dt = flyTime - *_lastDiveThetaCmdTime;
            const double filterAlpha = dt / (std::max(0.0, config.thetaCmdFilterTimeConstant) + dt);
            thetaCmd = *_lastDiveThetaCmd + filterAlpha * (rawThetaCmd - *_lastDiveThetaCmd);
        }
        _lastDiveThetaCmd = thetaCmd;
        _lastDiveThetaCmdTime = flyTime;

        const double thetaError = thetaCmd - theta;
        const double headingError = wrapAngle(losMidcourse.sigma_az - psi);
        const double vyCmd = speed * std::sin(thetaCmd);

        Eigen::Vector3d diveAccCmd = Eigen::Vector3d::Zero();
        diveAccCmd.x() = 0.0;
        diveAccCmd.y() = 9.8 * std::cos(theta) + config.thetaGain * speed * thetaError - config.verticalDamping * (selfVelNue.y() - vyCmd);
        diveAccCmd.z() = -config.headingGain * speed * headingError;

        GCInfo gcInfo;
        gcInfo.phase = phase;
        gcInfo.losInfo = phase == GuidancePhase::DiveHandover || phase == GuidancePhase::DiveTerminal ? losTerminal : losMidcourse;
        gcInfo.acc_cmd_v = diveAccCmd;
        gcInfo.handoverRatio = 0.0;
        gcInfo.theta_cmd = thetaCmd;

        if (phase == GuidancePhase::DiveHandover || phase == GuidancePhase::DiveTerminal) {
            auto terminalAccCmd = guidance_pn(theta, losTerminal.sigma_az_dot, losTerminal.sigma_elv_dot, losTerminal.dis_dot);
            if (_lastTerminalAccCmd.has_value() && _lastTerminalAccCmdTime.has_value() && flyTime > *_lastTerminalAccCmdTime) {
                const double dt = flyTime - *_lastTerminalAccCmdTime;
                const Eigen::Vector3d rawDelta = terminalAccCmd - *_lastTerminalAccCmd;
                const double maxDelta = std::max(0.0, config.pnAccRateLimit) * dt;
                if (rawDelta.norm() > maxDelta && maxDelta > 0.0) {
                    terminalAccCmd = *_lastTerminalAccCmd + rawDelta.normalized() * maxDelta;
                }
                const double filterAlpha = dt / (std::max(0.0, config.pnFilterTimeConstant) + dt);
                terminalAccCmd = *_lastTerminalAccCmd + filterAlpha * (terminalAccCmd - *_lastTerminalAccCmd);
            }
            _lastTerminalAccCmd = terminalAccCmd;
            _lastTerminalAccCmdTime = flyTime;

            const double distanceBlend = computeBlendRatio(targetDis, config.handoverDistance, 0.0);
            const double timeBlend = std::isfinite(timeToGo) ? computeBlendRatio(timeToGo, config.terminalTime, 0.0) : 0.0;
            const double blendRatio = seekerLocked || phase == GuidancePhase::DiveTerminal ? std::max(distanceBlend, timeBlend) : 0.0;
            gcInfo.acc_cmd_v = (1.0 - blendRatio) * diveAccCmd + blendRatio * terminalAccCmd;
            gcInfo.handoverRatio = blendRatio;
        } else {
            _lastTerminalAccCmd = std::nullopt;
            _lastTerminalAccCmdTime = std::nullopt;
        }

        gcInfo.acc_cmd_v.x() = clamp(gcInfo.acc_cmd_v.x(), -9.8 * maxLoad, 9.8 * maxLoad);
        gcInfo.acc_cmd_v.y() = clamp(gcInfo.acc_cmd_v.y(), -9.8 * maxLoad, 9.8 * maxLoad);
        gcInfo.acc_cmd_v.z() = clamp(gcInfo.acc_cmd_v.z(), -9.8 * maxLoad, 9.8 * maxLoad);
        return gcInfo;
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

    GCInfo Guidance::getGCInfoAnalyticGlide(const double Mass, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, const double maxLoad) {
        return getGCInfoAnalyticMidcourse(0.0, targetPosEcf, targetVelEcf, state, Mass, maxLoad);
    }

    GCInfo Guidance::getGCInfoAnalyticMidcourse(const double flyTime, const Eigen::Vector3d &targetPosEcf, const Eigen::Vector3d &targetVelEcf, const State &state, const double mass,
                                                const double maxLoad) {
        (void) flyTime;

        constexpr double gravity = 9.80665;
        constexpr double minTerminalSpeed = 900.0;
        constexpr double maxTerminalSpeed = 2200.0;
        constexpr double minDrag = 0.05;
        constexpr double scaleHeight = 7200.0;

        GCInfo gc_info;
        gc_info.phase = GuidancePhase::Glide;
        gc_info.losInfo = getLOSInfo(targetPosEcf, targetVelEcf, state);

        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto targetLla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(targetPosEcf);
        const auto velNue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const auto relNue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf, lla.x(), lla.y()) -
                            ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(state.posEcf, lla.x(), lla.y());

        const double speed = std::max(velNue.norm(), 1.0);
        const double rangeToGo = std::max(Eigen::Vector2d(relNue.x(), relNue.z()).norm(), 1000.0);
        const double theta = ModelDevelop::Utils::CoordinateHelper::getTheta(velNue);
        const double psi = ModelDevelop::Utils::CoordinateHelper::getPsi(velNue);
        const double headingError = wrapAngle(gc_info.losInfo.sigma_az - psi);

        const double terminalSpeedUpper = std::max(minTerminalSpeed, std::min(maxTerminalSpeed, speed - 10.0));
        const double terminalSpeed = clamp(targetVelEcf.norm() > 100.0 ? targetVelEcf.norm() : speed * 0.65, minTerminalSpeed, terminalSpeedUpper);
        const double currentDrag = std::max(estimateDragAcceleration(state, mass), minDrag);
        const double terminalRho = ModelDevelop::Utils::Aerodynamics::calculateAtmosphereDensity(std::max(targetLla.z(), 0.0));
        const double terminalDrag = std::max(0.5 * terminalRho * terminalSpeed * terminalSpeed * 0.223 * 0.08 / std::max(mass, 1.0), minDrag);
        const double dc = solveDragProfileMidpoint(speed, terminalSpeed, currentDrag, terminalDrag, rangeToGo);
        const double refDrag = dragProfile(speed, speed, terminalSpeed, currentDrag, dc, terminalDrag);
        const double refDragSlope = dragProfileSlope(speed, speed, terminalSpeed, currentDrag, dc, terminalDrag);
        const double refDragDot = -refDragSlope * refDrag;

        const double refRho = clamp(2.0 * std::max(mass, 1.0) * refDrag / (speed * speed * 0.223 * 0.08), 1.0e-6, 1.225);
        const double dragAltitudeRef = -scaleHeight * std::log(refRho / 1.225);
        const double lineAltitudeRef = targetLla.z() + (lla.z() - targetLla.z()) * clamp(rangeToGo / std::max(rangeToGo + 50000.0, 1.0), 0.0, 1.0);
        const double altitudeRef = 0.65 * dragAltitudeRef + 0.35 * lineAltitudeRef;

        const double altitudeError = clamp(altitudeRef - lla.z(), -20000.0, 20000.0);
        const double dragError = clamp(currentDrag - refDrag, -20.0, 20.0);
        const double thetaRef = clamp(std::atan2(targetLla.z() - lla.z(), rangeToGo) + 0.018 * dragError, -25.0 / 57.3, 15.0 / 57.3);
        const double thetaDotCmd = 0.75 * wrapAngle(thetaRef - theta) + 0.00035 * altitudeError - 0.0005 * velNue.y() - 0.015 * refDragDot;

        const double liftAcc = estimateLiftAcceleration(state, mass, maxLoad);
        const double verticalAcc = clamp(gravity * std::cos(theta) + speed * thetaDotCmd, -gravity * maxLoad, gravity * maxLoad);
        const double cosSigma = clamp((verticalAcc - gravity * std::cos(theta)) / std::max(liftAcc, gravity), -1.0, 1.0);
        const double bankMagnitude = std::acos(cosSigma);

        const double corridor = clamp((3.0 + 17.0 * rangeToGo / 200000.0) / 57.3, 2.0 / 57.3, 25.0 / 57.3);
        if (std::fabs(headingError) >= corridor) {
            lastBankSign = guidanceHeadingSign(headingError, corridor);
        }
        const double sign = std::fabs(headingError) < 0.15 * corridor ? 0.0 : lastBankSign;
        const double headingRatio = clamp(std::fabs(headingError) / corridor, 0.0, 1.0);
        const double lateralAcc = sign * headingRatio * std::min(liftAcc * std::sin(bankMagnitude), gravity * maxLoad);

        gc_info.acc_cmd_v = Eigen::Vector3d(0.0, verticalAcc, clamp(lateralAcc, -gravity * maxLoad, gravity * maxLoad));
        gc_info.theta_cmd = thetaRef;
        return gc_info;
    }

    double Guidance::estimateDragAcceleration(const State &state, const double mass) const {
        constexpr double referenceArea = 0.223;
        double alpha = 0.0;
        double beta = 0.0;
        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto velNue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        ModelDevelop::Utils::CoordinateHelper::calculateAngleOfAttack(velNue, state.qbn, alpha, beta);

        const double speed = velNue.norm();
        const double ma = std::max(speed / av, 0.1);
        const double ca0 = 0.03 + 0.0005 * ma;
        const double k = 0.8 + 0.0005 * ma;
        const double cd = ca0 + k * (alpha * alpha + beta * beta);
        const double rho = ModelDevelop::Utils::Aerodynamics::calculateAtmosphereDensity(lla.z());
        return 0.5 * rho * speed * speed * referenceArea * cd / std::max(mass, 1.0);
    }

    double Guidance::estimateLiftAcceleration(const State &state, const double mass, const double maxLoad) const {
        constexpr double referenceArea = 0.223;
        constexpr double maxAlpha = 20.0 / 57.3;
        const auto lla = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto velNue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());
        const double speed = velNue.norm();
        const double ma = std::max(speed / av, 0.1);
        const double cn = 0.3 + 0.6 * ma * ma / (1.0 + 0.8 * ma * ma * ma * ma) + 4.0 / std::sqrt(1.0 + (ma * ma - 1.0) * (ma * ma - 1.0));
        const double rho = ModelDevelop::Utils::Aerodynamics::calculateAtmosphereDensity(lla.z());
        const double availableLift = 0.5 * rho * speed * speed * referenceArea * cn * maxAlpha / std::max(mass, 1.0);
        return clamp(availableLift, 9.80665, 9.80665 * maxLoad);
    }

    double Guidance::solveDragProfileMidpoint(const double v0, const double vf, const double d0, const double df, const double rangeToGo) const {
        double dc = std::max(0.5 * (d0 + df), 0.05);
        for (int i = 0; i < 8; ++i) {
            const double range = profileRange(v0, vf, d0, dc, df);
            const double delta = std::max(0.02 * dc, 0.01);
            const double gradient = (profileRange(v0, vf, d0, dc + delta, df) - range) / delta;
            if (std::fabs(gradient) < 1.0e-6) {
                break;
            }
            dc = clamp(dc + 0.5 * (rangeToGo - range) / gradient, 0.05, 80.0);
        }
        return dc;
    }

    double Guidance::dragProfile(const double v, const double v0, const double vf, const double d0, const double dc, const double df) const {
        const double vm = 0.5 * (v0 + vf);
        const double l0 = (v - vm) * (v - vf) / ((v0 - vm) * (v0 - vf));
        const double l1 = (v - v0) * (v - vf) / ((vm - v0) * (vm - vf));
        const double l2 = (v - v0) * (v - vm) / ((vf - v0) * (vf - vm));
        return std::max(0.05, d0 * l0 + dc * l1 + df * l2);
    }

    double Guidance::dragProfileSlope(const double v, const double v0, const double vf, const double d0, const double dc, const double df) const {
        constexpr double dv = 1.0;
        return (dragProfile(v + dv, v0, vf, d0, dc, df) - dragProfile(v - dv, v0, vf, d0, dc, df)) / (2.0 * dv);
    }

    double Guidance::profileRange(const double v0, const double vf, const double d0, const double dc, const double df) const {
        if (v0 <= vf + 1.0) {
            return 0.0;
        }
        constexpr int segments = 64;
        const double dv = (v0 - vf) / segments;
        double range = 0.0;
        for (int i = 0; i < segments; ++i) {
            const double va = v0 - i * dv;
            const double vb = va - dv;
            const double vm = 0.5 * (va + vb);
            range += vm / dragProfile(vm, v0, vf, d0, dc, df) * dv;
        }
        return range;
    }

    double Guidance::guidanceHeadingSign(const double headingError, const double corridor) {
        if (std::fabs(headingError) < 0.15 * corridor) {
            return 0.0;
        }
        return headingError > 0.0 ? 1.0 : -1.0;
    }

    double Guidance::smoothStep(const double ratio) {
        const double x = clamp(ratio, 0.0, 1.0);
        return x * x * (3.0 - 2.0 * x);
    }

    void Guidance::reset() {
        boostInitialPsi = std::nullopt;
        lastBankSign = 1.0;
        _lastDiveThetaCmd = std::nullopt;
        _lastDiveThetaCmdTime = std::nullopt;
        _lastDivePhase = std::nullopt;
        _lastTerminalAccCmd = std::nullopt;
        _lastTerminalAccCmdTime = std::nullopt;
    }

}

#undef PRETTY_FILE_NAME
