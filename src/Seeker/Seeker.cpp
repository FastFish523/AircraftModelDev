//
// Created by Administrator on 2026/3/12.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "SEEKER/Seeker.h"
#include "CoordinateHelper.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/SEEKER/SEEKER"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::SEEKER {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
    LosInfo Seeker::getLOSInfo(const Eigen::Vector3d& targetPosEcf, const Eigen::Vector3d& targetVelEcf, const State& state) {
        const Eigen::Vector3d lla     = ModelDevelop::Utils::CoordinateHelper::ecefToLla(state.posEcf);

        const auto target_position_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(targetPosEcf,lla.x(),lla.y());
        const auto position_nue        = ModelDevelop::Utils::CoordinateHelper::ecefToNuePosition(state.posEcf,lla.x(),lla.y());
        const auto target_velocity_nue = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(targetVelEcf, lla.x(), lla.y());
        const auto velocity_nue        = ModelDevelop::Utils::CoordinateHelper::ecefToNueVelocity(state.velEcf, lla.x(), lla.y());


        const auto rel_pos = (target_position_nue - position_nue).eval();
        const auto rel_vel = (target_velocity_nue - velocity_nue).eval();
        const auto rel_w   = (rel_pos.cross(rel_vel) / rel_pos.squaredNorm()).eval();
        const auto dis     = (target_position_nue - position_nue).norm();

        const auto theta = ModelDevelop::Utils::CoordinateHelper::getTheta(velocity_nue);
        const auto psi   = ModelDevelop::Utils::CoordinateHelper::getPsi(velocity_nue);

        const auto sigma_az_dot = -rel_w.x() * sin(theta) * cos(psi) + rel_w.y() * cos(theta) + rel_w.z() * sin(theta) *sin(psi);
        const auto sigma_elv_dot  = rel_w.x() * sin(psi) + rel_w.z() * cos(psi);

        const auto dis_dot = rel_pos.dot(rel_vel) / dis;

        _solver.update(ModelDevelop::Utils::CoordinateHelper::getTheta(rel_pos),ModelDevelop::Utils::CoordinateHelper::getPsi(rel_pos));
        const auto rel_pos_body = Utils::CoordinateHelper::nueToBodyVector(rel_pos, state.qbn);
        const auto elv_body = Utils::CoordinateHelper::getTheta(rel_pos_body);
        const auto az_body = Utils::CoordinateHelper::getPsi(rel_pos_body);



        LosInfo los_info{};
        los_info.dis_dot = velocity_nue.norm();
        los_info.sigma_az_dot = _solver.getYawRate();
        los_info.sigma_elv_dot = _solver.getPitchRate();
        los_info.sigma_elv = ModelDevelop::Utils::CoordinateHelper::getTheta(rel_pos);
        los_info.sigma_az = ModelDevelop::Utils::CoordinateHelper::getPsi(rel_pos);
        los_info.sigma_az_b = az_body;
        los_info.sigma_elv_b = elv_body;

        return los_info;
    }
// endregion

// region Get/Set选择器
// endregion

// region Private Methods
// endregion
}
#undef PRETTY_FILE_NAME