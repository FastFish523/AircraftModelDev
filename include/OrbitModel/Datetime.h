#pragma once
#include "stdafx.h"
#include "Tools.h"
namespace JTC_Basic_OrbitModel {
#define isPrime(year) ((year%4==0&&year%100!=0)||(year%400==0))
    class  Datetime {
    public:
        Datetime();

        ~Datetime();

        //! MJD转化为日期时间
        double MJD01(int year, int month, int day, int hour, int minute, double second);

        void GetCalendar01(double mjd, int &Y, int &M, int &D, int &h, int &min, double &s);


        double MJD(const datetime &i_datetime);

        void GetCalendar(double mjd, datetime &i_datetime);

        //datetime GetCalendar(double jd);
        datetime GetCalendar1(double jd);

        void Cal_sun(double *rsun, double mjd);

        int dateDiff(datetime min_date, datetime max_date);

        datetime formatSecond(double secs);

        Tools tools;

    private:
        int n_day;
        double sec;
    };

}
