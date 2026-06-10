//
// Created by Administrator on 2026/1/28.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "HACM/Engine.h"
#include <valarray>
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/HACM/HACM"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::HACM {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
    EigenInfo Engine::getEigenInfo(const double simStep,const double flyTime, const State& state,double desired_vel, const double dx, const double dy, const double dz) {
        const double V= state.velEcf.norm();
        double FF = 1000 *(desired_vel-V);
        if(FF<=0) {
            FF=0;
        }
        if(FF>100*1000) {
            FF = 100*1000;
        }
        if(m<=0) {
            m=0;
        }
        double dm = FF/Isp/9.8;
        if(flyTime<5) {
            FF = 500*1000;
            dm=0;
        }

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