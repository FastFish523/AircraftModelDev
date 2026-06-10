//
// Created by Administrator on 2026/3/16.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "OrbitModel/FileSaver.h"
#include <filesystem>
#include "OrbitModel/JTC_OrbitModel.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/OrbitModel/OrbitModel"
// endregion

// region Using NameSpace

// endregion

namespace JTC_Basic_OrbitModel {
// region Static Attributes Init
// endregion

// region USING/FRIEND
// endregion

    FileSaver::FileSaver(const std::string& path) {
        _directory = path;
    }

// region Constructor
    FileSaver::~FileSaver() {
        if (result_fp_traj() != nullptr) {
            fclose(result_fp_traj());
        }
    }
// endregion

// region Public Methods
    void FileSaver::save_traj(const JTC_OrbitModel *missile) {
        fprintf(
            result_fp_traj(),
            "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
            missile->flyTime(),
            missile->positionLaunchNUE().x(),
            missile->positionLaunchNUE().y(),
            missile->positionLaunchNUE().z(),
            missile->V(),
            missile->attitudeEuler().x(),
            missile->attitudeEuler().y(),
            missile->attitudeEuler().z(),
            missile->velocityTheta(),
            missile->velocityPsi(),
            missile->alpha(),
            missile->beta(),
            missile->accelerationBody().x(),
            missile->accelerationBody().y(),
            missile->accelerationBody().z(),
            missile->mass(),
            missile->P(),
            missile->acc_cmd_b_y(),
            missile->acc_cmd_b_z(),
            missile->targetPositionLaunchNUE().x(),
            missile->targetPositionLaunchNUE().y(),
            missile->targetPositionLaunchNUE().z(),
            missile->rudder().x(),
            missile->rudder().y(),
            missile->rudder().z(),
            missile->velocityNUE().x(),
            missile->velocityNUE().y(),
            missile->velocityNUE().z(),
            missile->w_xyz().x(),
            missile->w_xyz().y(),
            missile->w_xyz().z(),
            missile->lla().x(),
            missile->lla().y(),
            missile->lla().z(),
            missile->sigmaElv(),
            missile->sigmaAz(),
            missile->sigmaElvDot(),
            missile->sigmaAzDot()
        );
    }

// endregion

// region Get/Set选择器
// endregion

// region Private Methods
    auto FileSaver::result_fp_traj() -> FILE * {
        if (!std::filesystem::exists(_directory)) {
            std::filesystem::create_directories(_directory);
        }
        std::string filename = _directory + "result.dat";
        if (!fp_traj) {
            fp_traj = fopen(filename.c_str(), "w");
            if (!fp_traj) {
                printf("错误：无法打开文件\n");
                exit(-1);
            }
        }
        return fp_traj;
    }

// endregion
}
#undef PRETTY_FILE_NAME