//
// Created by MikuSoft on 2026/5/7.
// Copyright (c) 2026 JiuTianAoXiang All rights reserved.
//
#include <OrbitModel/FileSaver.h>

#include "Util/DateTime.h"
#include "include/OrbitModel/JTC_OrbitModel.h"

int main(){
    ModelDevelop::Utils::DateTime dateTime(2000,1,1,0,0,0);
    JTC_Basic_OrbitModel::JTC_OrbitModel orbitModel(dateTime.year(),dateTime.month(),dateTime.day(),dateTime.hour(),dateTime.minute(),dateTime.second()+dateTime.millisecond()*0.001,
        6878000,0.0,90,0,0,0);
    for(int i=0;i<3600;i++){
        orbitModel.update(dateTime.year(),dateTime.month(),dateTime.day(),dateTime.hour(),dateTime.minute(),dateTime.second()+dateTime.millisecond()*0.001,1);
        dateTime.addSeconds(1);
    }
    return 0;
}