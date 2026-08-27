//
// Created by Codex on 2026/5/26.
//

#pragma once

#include "CommonStructs.h"
#include "State.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace ModelDevelop::HTV2 {
    struct TerminalViewConstraint {
        Eigen::Vector3d pointBody{0.0, 0.0, 0.0};
        Eigen::Vector3d boresightBody{1.0, 0.0, 0.0};
        bool useMountAngles = false;
        double mountAzDeg = 0.0;//方位角，水平面内从 +X 向 +Z 偏转
        double mountElDeg = 0.0;//高低角，从 X-Z 平面向 +Y 抬起
        double coneAngleDeg = 0.0;
        double reacquireConeAngleDeg = 0.0;
        bool releaseInsideCone = false;
    };

    struct TerminalAttitudeHoldConfig {
        bool enable = false;
        double startDistance = 30000.0;
        double duration = 10.0;
        double rollDeg = 0.0;
        bool holdInitialAttitude = false;
        Eigen::Vector3d aimAxisBody{1.0, 0.0, 0.0};
        TerminalViewConstraint view{};
        Eigen::Vector3d kp{5000.0, 800000.0, 5000.0};
        Eigen::Vector3d ki{0.0, 0.0, 0.0};
        Eigen::Vector3d kd{3000.0, 800000.0, 3000.0};
        Eigen::Vector3d integralLimit{0.05, 0.05, 0.05};
        double maxMoment = 20000.0;
    };

    struct TerminalAttitudeHoldOutput {
        bool phaseActive = false;
        bool active = false;
        bool insideCone = false;
        double viewAngleDeg = 0.0;
        Eigen::Vector3d attitudeCmdDeg{0.0, 0.0, 0.0};
        Eigen::Quaterniond attitudeCmd = Eigen::Quaterniond::Identity();
        Eigen::Vector3d momentBody{0.0, 0.0, 0.0};
    };

    class TerminalAttitudeHold {
    public:
        TerminalAttitudeHold() = default;

        void configure(const TerminalAttitudeHoldConfig &config);

        [[nodiscard]] bool holdInitialAttitude() const;

        void reset();

        TerminalAttitudeHoldOutput update(
            double flyTime,
            double targetDistance,
            const Eigen::Vector3d &targetPosEcf,
            const State &state,
            const ImuInfo &imuInfo
        );

    private:
        [[nodiscard]] bool shouldStart(double targetDistance) const;

        [[nodiscard]] bool isWithinHoldWindow(double flyTime) const;

        [[nodiscard]] Eigen::Vector3d computeLookAtEulerDeg(
            const Eigen::Vector3d &targetPosEcf,
            const State &state
        ) const;

        [[nodiscard]] Eigen::Quaterniond computeTargetAttitude(
            const Eigen::Vector3d &targetDirectionNue,
            const State &state
        ) const;

        [[nodiscard]] Eigen::Vector3d computeTargetDirectionNue(
            const Eigen::Vector3d &targetPosEcf,
            const State &state
        ) const;

        [[nodiscard]] Eigen::Vector3d computeBoresightBody() const;

        [[nodiscard]] Eigen::Vector3d computeBoresightNue(const State &state) const;

        [[nodiscard]] double computeViewAngleDeg(
            const Eigen::Vector3d &targetDirectionNue,
            const State &state
        ) const;

        [[nodiscard]] Eigen::Vector3d computeAttitudeMoment(
            const Eigen::Quaterniond &attitudeCmd,
            const State &state,
            const ImuInfo &imuInfo,
            double dt
        );

        void resetIntegral();

        static double limit(double value, double lower, double upper);

    private:
        TerminalAttitudeHoldConfig _config{};
        bool _started = false;
        bool _releasedInsideCone = false;
        bool _hasLastMomentTime = false;
        double _lastMomentTime = 0.0;
        Eigen::Vector3d _integralAttitudeErrorBody{0.0, 0.0, 0.0};
        double _startTime = 0.0;
    };
}
