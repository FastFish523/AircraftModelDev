//
// Created by Administrator on 2026/1/30.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "GPI/Engine.h"
#include <valarray>
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/GPI/GPI"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::GPI {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
    EigenInfo Engine::getEigenInfo(const double simStep, double flyTime, const State &state, const double launch_v_body, const double dx, const double dy, const double dz)  {
        const double V= state.velEcf.norm();
        double FF = 1000*(launch_v_body-V);

        if(FF>8000) {
            FF = 8000;
        }
        if(FF<=0) {
            FF=0;
        }

        const double dm = FF/Isp/9.8;
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