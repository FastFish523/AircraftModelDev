//
// Created by Administrator on 2026/3/12.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include <utility>

#include "BGM/Seeker.h"
#include "CoordinateHelper.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/BGM/BGM"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::BGM{
    LosInfo Seeker::getLOSInfo(const Eigen::Vector3d& targetPosEcf, const Eigen::Vector3d& targetVelEcf,
                               const State& state){
        return myGetLOSInfoFunction(targetPosEcf, targetVelEcf, state);
    }

    void Seeker::setGetLOSInfoFunction(GetLOSInfoFunction getLOSInfoFunction){
        myGetLOSInfoFunction = getLOSInfoFunction;
    }
}
#undef PRETTY_FILE_NAME
