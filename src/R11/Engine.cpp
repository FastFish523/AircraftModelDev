//
// Created by Administrator on 2026/2/2.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "R11/Engine.h"
#include <valarray>
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
    EigenInfo Engine::getEigenInfo(const double simStep,const double flyTime, const State& state, const double dx, const double dy, const double dz) {
        double t1 = dT1_up;
        double t2=dT1_up+T1;
        double t3=dT1_up+T1+dT1_down;
        double t4=dT1_up+T1+dT1_down+T2;
        double t5=dT1_up+T1+dT1_down+T2+dT2_down;
        double t = flyTime;
        double FF=0;
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