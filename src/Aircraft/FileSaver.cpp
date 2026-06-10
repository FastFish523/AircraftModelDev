//
// Created by Administrator on 2026/2/28.
//

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "Aircraft/FileSaver.h"
#include <filesystem>
#include "Aircraft/AircraftModel.h"
// endregion
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/Aircraft/Aircraft"
// endregion

// region Using NameSpace

// endregion

namespace ModelDevelop::Aircraft {
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
        if (result_fp_aero() != nullptr) {
            fclose(result_fp_aero());
        }
    }
// endregion

// region Public Methods
    void FileSaver::save_traj(const AircraftModel *m_model) {
        fprintf(
            result_fp_traj(),
            "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
            m_model->flyTime(),
            m_model->positionLaunchNUE().x(),
            m_model->positionLaunchNUE().y(),
            m_model->positionLaunchNUE().z(),
            m_model->V(),
            m_model->attitudeEuler().x(),
            m_model->attitudeEuler().y(),
            m_model->attitudeEuler().z(),
            m_model->velocityTheta(),
            m_model->velocityPsi(),
            m_model->alpha(),
            m_model->beta(),
            m_model->accelerationBody().x(),
            m_model->accelerationBody().y(),
            m_model->accelerationBody().z(),
            m_model->mass(),
            m_model->P(),
            m_model->acc_cmd_b_y(),
            m_model->acc_cmd_b_z(),
            m_model->targetPositionLaunchNUE().x(),
            m_model->targetPositionLaunchNUE().y(),
            m_model->targetPositionLaunchNUE().z(),
            m_model->rudder().x(),
            m_model->rudder().y(),
            m_model->rudder().z(),
            m_model->velocityNUE().x(),
            m_model->velocityNUE().y(),
            m_model->velocityNUE().z(),
            m_model->w_xyz().x(),
            m_model->w_xyz().y(),
            m_model->w_xyz().z(),
            m_model->lla().x(),
            m_model->lla().y(),
            m_model->lla().z(),
            m_model->sigmaElv(),
            m_model->sigmaAz(),
            m_model->sigmaElvDot(),
            m_model->sigmaAzDot()
        );
    }

    void FileSaver::save_aero(const AircraftModel *m_model) {
        const auto [a11, a12, a13, a14, a15, a16,
            a21, a22, a23, a24, a25, a26,
            a31, a32, a33, a34, a35, a36,
            b11, b12, b13, b14, b15, b16, b17, b18,
            b21, b22, b23, b24, b25, b26, b27, b28,
            b31, b32, b33, b34, b35, b36, b37, b38] = m_model->derivative();

        fprintf(result_fp_aero(), "%.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f"
                " %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f"
                "\n",
                m_model->flyTime(),
                a11, a12, a13, a14, a15, a16,
                a21, a22, a23, a24, a25, a26,
                a31, a32, a33, a34, a35, a36,
                m_model->V(),
                b11, b12, b13, b14, b15, b16, b17, b18,
                b21, b22, b23, b24, b25, b26, b27, b28,
                b31, b32, b33, b34, b35, b36, b37, b38
        );
    }
// endregion

// region Get/Set选择器
// endregion

// region Private Methods
    auto FileSaver::result_fp_aero() -> FILE * {
        if (!std::filesystem::exists(_directory)) {
            std::filesystem::create_directories(_directory);
        }
        std::string filename = _directory + "air.dat";
        if (!fp_aero) {
            fp_aero = fopen(filename.c_str(), "w");
            if (!fp_aero) {
                printf("错误：无法打开文件\n");
                exit(-1);
            }
        }
        return fp_aero;
    }

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