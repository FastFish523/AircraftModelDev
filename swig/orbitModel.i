%module orbitModelPy
%{
#include "../include/OrbitModel/JTC_OrbitModel.h"
%}

%include "std_string.i"
namespace JTC_Basic_OrbitModel{
        class JTC_OrbitModel{
            public:
                /*轨道根数构造*/
                JTC_OrbitModel(int year, int month, int day, int hour, int minute, double second,
                double semi_majorAxis, double eccentricity, double inclination, double argOfPerigee, double rann,
                double meananom);


                /* 计算给定时刻的位置速度
                返回值（18个数据，地惯系下位置速度，地固系下位置速度，经纬高位置速度）*/
                void update(int year, int month, int day, int hour, int minute, double second, double step);
        };
};
