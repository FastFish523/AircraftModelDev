//
// Created by Administrator on 2026/3/16.
//


#pragma once 

// region Include
// region STL
// endregion
// region ThirdParty
// endregion
// region Self
#include "CommonStructs.h"
#include <cstdio>
#include <string>
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDevelop/OrbitModel/OrbitModel"
// endregion

namespace JTC_Basic_OrbitModel {
    class JTC_OrbitModel;
    class FileSaver {
// region USING/FRIEND
    private:
// endregion

// region Constructor
    public:
        explicit FileSaver(const std::string& path);

        ~FileSaver() ;

// endregion

// region Public Attributes
    public:
// endregion

// region Public Methods
    public:
        void save_traj(const JTC_OrbitModel *missile);

// endregion

// region Get/Set选择器
    public:
// endregion

// region Private Attributes
    private:
        /*!
         * @brief 文件存储
         */
        std::string _directory = {};
        /*!
         * @brief 弹道文件的指针
         */
        FILE *fp_traj = nullptr;
// endregion

// region Private Methods
    private:
        auto result_fp_traj() -> FILE *;

// endregion
    };
}
#undef PRETTY_FILE_NAME