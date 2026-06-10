//
// Created by Codex on 2026/5/26.
//

#include "R11/TerminalAttitudeHold.h"

#include <cmath>

#include "CoordinateHelper.h"

namespace {
    constexpr double RAD_TO_DEG = 57.29577951308232;
    constexpr double DEG_TO_RAD = 0.017453292519943295;
}

namespace ModelDevelop::R11 {
    void TerminalAttitudeHold::configure(const TerminalAttitudeHoldConfig &config) {
        _config = config;
        if (!_config.view.useMountAngles &&
            (_config.view.boresightBody - Eigen::Vector3d{1.0, 0.0, 0.0}).norm() < 1e-9 &&
            (_config.aimAxisBody - Eigen::Vector3d{1.0, 0.0, 0.0}).norm() >= 1e-9) {
            _config.view.boresightBody = _config.aimAxisBody;
        }
        reset();
    }

    void TerminalAttitudeHold::reset() {
        _started = false;
        _releasedInsideCone = false;
        resetIntegral();
        _startTime = 0.0;
    }

    TerminalAttitudeHoldOutput TerminalAttitudeHold::update(
        const double flyTime,
        const double targetDistance,
        const Eigen::Vector3d &targetPosEcf,
        const State &state,
        const ImuInfo &imuInfo
    ) {
        TerminalAttitudeHoldOutput output{};
        if (!_config.enable) {
            return output;
        }

        if (!_started && shouldStart(targetDistance)) {
            _started = true;
            _releasedInsideCone = false;
            _startTime = flyTime;
        }

        const auto targetDirectionNue = computeTargetDirectionNue(targetPosEcf, state);
        output.viewAngleDeg = computeViewAngleDeg(targetDirectionNue, state);
        output.insideCone = output.viewAngleDeg <= _config.view.coneAngleDeg;
        if (!_started || !isWithinHoldWindow(flyTime)) {
            resetIntegral();
            return output;
        }

        output.phaseActive = true;
        if (_config.view.releaseInsideCone) {
            const double reacquireConeAngleDeg =
                _config.view.reacquireConeAngleDeg > _config.view.coneAngleDeg
                    ? _config.view.reacquireConeAngleDeg
                    : _config.view.coneAngleDeg;

            if (_releasedInsideCone) {
                if (output.viewAngleDeg > reacquireConeAngleDeg) {
                    _releasedInsideCone = false;
                }
            } else if (output.insideCone) {
                _releasedInsideCone = true;
            }

            if (_releasedInsideCone) {
                resetIntegral();
                return output;
            }
        }

        output.active = true;
        const auto attitudeCmd = computeTargetAttitude(targetDirectionNue, state);
        output.attitudeCmdDeg = Utils::CoordinateHelper::quaternionToEuler231(attitudeCmd);
        double dt = 0.0;
        if (_hasLastMomentTime) {
            dt = flyTime - _lastMomentTime;
        }
        _lastMomentTime = flyTime;
        _hasLastMomentTime = true;
        output.momentBody = computeAttitudeMoment(attitudeCmd, state, imuInfo, dt);
        return output;
    }

    bool TerminalAttitudeHold::shouldStart(const double targetDistance) const {
        return targetDistance > 0.0 && targetDistance <= _config.startDistance;
    }

    bool TerminalAttitudeHold::isWithinHoldWindow(const double flyTime) const {
        return flyTime - _startTime <= _config.duration;
    }

    Eigen::Vector3d TerminalAttitudeHold::computeLookAtEulerDeg(
        const Eigen::Vector3d &targetPosEcf,
        const State &state
    ) const {
        const auto relNue = computeTargetDirectionNue(targetPosEcf, state);
        const double yawDeg = Utils::CoordinateHelper::getPsi(relNue) * RAD_TO_DEG;
        const double pitchDeg = Utils::CoordinateHelper::getTheta(relNue) * RAD_TO_DEG;
        return {yawDeg, pitchDeg, _config.rollDeg};
    }

    Eigen::Quaterniond TerminalAttitudeHold::computeTargetAttitude(
        const Eigen::Vector3d &targetDirectionNue,
        const State &state
    ) const {
        const auto boresightBody = computeBoresightBody();

        const auto targetDirNue = targetDirectionNue.normalized().eval();
        const auto currentBoresightNue = Utils::CoordinateHelper::bodyToNueVector(boresightBody, state.qbn).normalized().eval();
        const auto alignCurrentToTarget = Eigen::Quaterniond::FromTwoVectors(currentBoresightNue, targetDirNue);
        auto attitudeCmd = alignCurrentToTarget * state.qbn;
        const Eigen::AngleAxisd clockAngle(_config.rollDeg * DEG_TO_RAD, targetDirNue);
        attitudeCmd = clockAngle * attitudeCmd;
        attitudeCmd.normalize();
        return attitudeCmd;
    }
    //从安装点计算目标方向
    Eigen::Vector3d TerminalAttitudeHold::computeTargetDirectionNue(
        const Eigen::Vector3d &targetPosEcf,
        const State &state
    ) const {
        const auto lla = Utils::CoordinateHelper::ecefToLla(state.posEcf);
        const auto pointOffsetNue = Utils::CoordinateHelper::bodyToNueVector(_config.view.pointBody, state.qbn);
        const auto pointOffsetEcf = Utils::CoordinateHelper::nueToEcefVector(pointOffsetNue, lla.x(), lla.y());
        const auto pointEcf = (state.posEcf + pointOffsetEcf).eval();
        auto relNue = Utils::CoordinateHelper::ecefToNueVector(targetPosEcf - pointEcf, lla.x(), lla.y());
        if (relNue.norm() < 1e-9) {
            relNue = computeBoresightNue(state);
        }
        return relNue;
    }
    //计算当前视轴
    Eigen::Vector3d TerminalAttitudeHold::computeBoresightBody() const {
        if (_config.view.useMountAngles) {
            const double az = _config.view.mountAzDeg * DEG_TO_RAD;
            const double el = _config.view.mountElDeg * DEG_TO_RAD;
            return Eigen::Vector3d{
                std::cos(el) * std::cos(az),
                std::sin(el),
                std::cos(el) * std::sin(az)
            }.normalized();
        }

        return _config.view.boresightBody.norm() > 1e-9
            ? _config.view.boresightBody.normalized().eval()
            : Eigen::Vector3d{1.0, 0.0, 0.0};
    }

    Eigen::Vector3d TerminalAttitudeHold::computeBoresightNue(const State &state) const {
        const auto boresightBody = computeBoresightBody();
        return Utils::CoordinateHelper::bodyToNueVector(boresightBody, state.qbn);
    }
    //视场角
    double TerminalAttitudeHold::computeViewAngleDeg(
        const Eigen::Vector3d &targetDirectionNue,
        const State &state
    ) const {
        const auto boresightNue = computeBoresightNue(state).normalized().eval();
        const auto targetDirNue = targetDirectionNue.normalized().eval();
        const double cosAngle = limit(boresightNue.dot(targetDirNue), -1.0, 1.0);
        return std::acos(cosAngle) * RAD_TO_DEG;
    }

    Eigen::Vector3d TerminalAttitudeHold::computeAttitudeMoment(
        const Eigen::Quaterniond &attitudeCmd,
        const State &state,
        const ImuInfo &imuInfo,
        const double dt
    ) {
        auto qError = attitudeCmd * state.qbn.conjugate();
        qError.normalize();
        if (qError.w() < 0.0) {
            qError.coeffs() *= -1.0;
        }

        const Eigen::AngleAxisd angleAxis(qError);
        Eigen::Vector3d attitudeErrorNue = angleAxis.axis() * angleAxis.angle();
        if (!std::isfinite(attitudeErrorNue.x()) || !std::isfinite(attitudeErrorNue.y()) || !std::isfinite(attitudeErrorNue.z())) {
            attitudeErrorNue.setZero();
        }

        const auto attitudeErrorBody = Utils::CoordinateHelper::nueToBodyVector(attitudeErrorNue, state.qbn);
        if (dt > 0.0 && dt < 1.0) {
            Eigen::Vector3d candidateIntegral = _integralAttitudeErrorBody + attitudeErrorBody * dt;
            for (int i = 0; i < 3; ++i) {
                const double integralLimit = std::abs(_config.integralLimit[i]);
                candidateIntegral[i] = limit(candidateIntegral[i], -integralLimit, integralLimit);
            }

            const Eigen::Vector3d candidateMoment =
                _config.kp.cwiseProduct(attitudeErrorBody) +
                _config.ki.cwiseProduct(candidateIntegral) -
                _config.kd.cwiseProduct(imuInfo.imu_w_xyz_body);

            for (int i = 0; i < 3; ++i) {
                const bool saturated = std::abs(candidateMoment[i]) > _config.maxMoment;
                const bool stillWindingUp = candidateMoment[i] * attitudeErrorBody[i] > 0.0;
                if (!saturated || !stillWindingUp) {
                    _integralAttitudeErrorBody[i] = candidateIntegral[i];
                }
            }
        }

        Eigen::Vector3d moment =
            _config.kp.cwiseProduct(attitudeErrorBody) +
            _config.ki.cwiseProduct(_integralAttitudeErrorBody) -
            _config.kd.cwiseProduct(imuInfo.imu_w_xyz_body);
        moment.x() = limit(moment.x(), -_config.maxMoment, _config.maxMoment);
        moment.y() = limit(moment.y(), -_config.maxMoment, _config.maxMoment);
        moment.z() = limit(moment.z(), -_config.maxMoment, _config.maxMoment);
        return moment;
    }

    void TerminalAttitudeHold::resetIntegral() {
        _integralAttitudeErrorBody.setZero();
        _hasLastMomentTime = false;
        _lastMomentTime = 0.0;
    }

    double TerminalAttitudeHold::limit(const double value, const double lower, const double upper) {
        return value < lower ? lower : (value > upper ? upper : value);
    }
}
