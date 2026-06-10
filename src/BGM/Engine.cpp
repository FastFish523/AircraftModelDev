//
// Created by Administrator on 2026/3/12.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "BGM/Engine.h"
#include <valarray>
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/BGM/BGM"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::BGM {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

// region Constructor
// endregion

// region Public Methods
    EigenInfo Engine::getEigenInfo(const double simStep,const double flyTime, const State& state, const double dx, const double dy, const double dz) {
        double t1 = dT1_up;
        double t2=t1+T1;
        double t3=t2+dT1_down;

        double t = flyTime;
        double FF=0;
        double Isp = Isp0;
        if (t<t1)
            FF = F1/dT1_up*t;
        else if (t<t2)
            FF = F1;
        else if ( t<t3)
           FF = (F2-F1)/dT1_down*(t-t2)+F1;
        else{
            Isp = Isp1;
            FF = 1000*(_MH*340-state.velEcf.norm());
            if(FF>F2)
              FF = F2;
            if(FF<0)
              FF=0;
          }
        if (m<=0)
           FF = 0;

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