//
// Created by Administrator on 2026/4/9.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "TheoreticalModel/Engine.h"
#include <valarray>
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/TheoreticalModel/TheoreticalModel"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::TheoreticalModel {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
    EigenInfo Engine::getEigenInfo(const double simStep,const double flyTime, const State& state, const double dx, const double dy, const double dz) {
        double FF = 10000*(340*3-state.velEcf.norm());
        FF = std::clamp(FF,0.0, 245000.0);
        const double dm = 0;
        m -= dm*simStep;
        const Eigen::Vector3d P_Body={
            FF/2.0*std::cos(dx)*(std::cos(dy)+std::cos(dz)),
            FF/2.0*std::cos(dx)*sin(dy),
            FF/2.0*std::cos(dx)*sin(dz),
        };
        EigenInfo eigen_info;
        eigen_info.mass=m;
        eigen_info.P_body = P_Body;

        eigen_info.inertia <<jx,0,0,0,jy,0,0,0,jz;

        return eigen_info;

    }
// endregion

// region Get/Set选择器
// endregion

// region Private Methods
// endregion

}
#undef PRETTY_FILE_NAME