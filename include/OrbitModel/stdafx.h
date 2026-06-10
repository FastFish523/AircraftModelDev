// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>

namespace JTC_Basic_OrbitModel {
    using namespace std;

    typedef struct _coe {
        //! Semi-major Axis(km)
        double a;//长半轴长
        //! Eccentricity
        double e;//偏心率
        //! Inclination(rad)
        double i;//轨道倾角
        //! Right Ascension of the Ascending Node (RAAN)(rad)
        double o;//升交点赤经
        //! Argument of Perigee(rad)
        double w;//近地点幅角
        //! Mean Anomaly(rad)
        double M;//真近点角

    } coe;

    typedef struct _datetime {
        int year;
        int month;
        int day;
        int hour;
        int minute;
        //int second;
        double second;
    public:
        //转化为字符串
        std::string toString() {
            std::string str;
            char value[128];
            memset(value, 0, 128);
            std::sprintf(value, "%04d-%02d-%02d %02d:%02d:%02d", this->year, this->month, this->day, this->hour,
                      this->minute, int(this->second));
            str = value;
            return str;
        }

        //转化为秒数，从1970年到现在经过了多少秒
        long long from1970Second() const {
            tm tm_;                                    // 定义tm结构体。
            tm_.tm_year = this->year - 1900;                 // 年，由于tm结构体存储的是从1900年开始的时间，所以tm_year为int临时变量减去1900。
            tm_.tm_mon = this->month - 1;                    // 月，由于tm结构体的月份存储范围为0-11，所以tm_mon为int临时变量减去1。
            tm_.tm_mday = this->day;                         // 日。
            tm_.tm_hour = this->hour;                        // 时。
            tm_.tm_min = this->minute;                       // 分。
            tm_.tm_sec = (int) this->second;                       // 秒。
            tm_.tm_isdst = 0;                          // 非夏令时。
            time_t t_ = mktime(&tm_);
            return t_;
        }

        bool operator<(const _datetime &t) const   // 两个const是必需的。
        {
            long long tt1 = this->from1970Second();
            long long tt2 = t.from1970Second();
            if (tt1 < tt2)
                return true;
            return false;
        }
    } datetime;


//struct date
//{
//	int year;
//	int month;
//	int day;
//};



}
