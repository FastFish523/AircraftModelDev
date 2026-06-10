//
// Created by MikuSoft on 2026/1/28.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//

#pragma once
// region Self
// endregion
// endregion

// region Using NameSpace
// endregion

// region Define
#define PRETTY_FILE_NAME "ModelDev/PAC2/fileSaver"
#if defined(_WIN32) && !defined(StaticModel_Build)
#ifdef SharedModel_Build
#define Dll_Export_Import __declspec(dllexport)
#else
#define Dll_Export_Import __declspec(dllimport)
#endif
#else
#define Dll_Export_Import
#endif
#include <cstdio>
#include <string>



namespace ModelDev::PAC2 {
    class Missile;
    class Dll_Export_Import FileSaver {
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
        void save_traj(const Missile *missile);

        void save_aero(const Missile *missile);

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
        /*!
         * @brief 气动导数文件的指针
         */
        FILE *fp_aero = nullptr;
// endregion

// region Private Methods
    private:
        auto result_fp_aero() -> FILE *;

        auto result_fp_traj() -> FILE *;

// endregion
    };
}
#undef Dll_Export_Import
#undef PRETTY_FILE_NAME
