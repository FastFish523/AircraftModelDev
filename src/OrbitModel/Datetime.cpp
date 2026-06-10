#include "OrbitModel/Datetime.h"
#include <cmath>
#include <OrbitModel/Constant.h>

#include "OrbitModel/stdafx.h"
namespace JTC_Basic_OrbitModel {
    Datetime::Datetime() {
    }


    Datetime::~Datetime() {
    }

//! 日期时间转化为MJD
    double Datetime::MJD01(const int year, const int month, const int day, const int hour, const int minute, const double second) {
        return day - 32075 + 1461. * (year + 4800 + (month - 14.) / 12) / 4  //注意这里的除法为整除
               + 367 * (month - 2 - (month - 14.) / 12 * 12) / 12
               - 3 * ((year + 4900 + (month - 14.) / 12) / 100) / 4
               - 0.5 - 2400000.5 + static_cast<double>(hour) / 24.0
               + static_cast<double>(minute) / 1440.0 + static_cast<double>(second) / 86400.0;  //儒勒日数
    }

    void Datetime::GetCalendar01(double mjd, int &Y, int &M, int &D, int &h, int &min, double &s) {
        // 增加浮点数的处理
        // 当输入为55197.999999999985(应该是2010-1-1,23:59:59.999...或2010-1-2,0:0:0)时，
        // 发生计算错误，计算J时数字自动进位，导致计算的日期正确(2010-1-2),但是在计算s时使用的方法为
        // s = (mjd-floor(mjd))*86400
        // 这里对mjd取整时，并没有进位，计算出来的时间是23:59:59.999...，与日期拼在一起之后导致多算了一天

        int mjdI = static_cast<int>(floor(mjd));
        s        = (mjd - mjdI) * 86400;
        h        = static_cast<int>(floor(s / 3600));
        min      = static_cast<int>(floor((s - h * 3600) / 60));
        s        = s - h * 3600 - min * 60;
        s        = tools.round(s * 1e5) / 1e5;
        if (s == 60) {
            min++;
            s = 0;
            if (min == 60) {
                h++;
                min = 0;
                if (h == 24) {
                    mjdI++;
                    h = 0;
                }
            }
        }
        const int J  = static_cast<int>(floor(mjdI + 2400000.5 + 0.5));
        const int N  = 4 * (J + 68569) / 146097; //注意这里的除法为整除
        const int L1 = J + 68569 - (N * 146097 + 3) / 4;
        const int Y1 = 4000 * (L1 + 1) / 1461001;
        const int L2 = L1 + 31 - 1461 * Y1 / 4;
        const int M1 = 80 * L2 / 2447;
        D            = L2 - 2447 * M1 / 80;
        const int L3 = M1 / 11;
        M            = M1 + 2 - 12 * L3;
        Y            = 100 * (N - 49) + Y1 + L3;

    }

    double Datetime::MJD(const datetime &i_datetime) {
        const int year      = i_datetime.year;
        const int month     = i_datetime.month;
        const int day       = i_datetime.day;
        const int hour      = i_datetime.hour;
        const int minute    = i_datetime.minute;
        const double second = i_datetime.second;
        int y, m;
        if (month <= 2) {
            y = year - 1;
            m = month + 12;
        } else {
            y = year;
            m = month;
        }

        const long A = static_cast<long>(y / 100.);
        const long B = 2 - A + static_cast<long>(A / 4.);

        double jd = static_cast<long>(365.25 * y) + static_cast<long>(30.6001 * (m + 1)) + day +
                    1720994.5;

        jd += hour / 24. + minute / 1440. + second / 86400.;

        if (year > 1583) return (jd + B);
        else return (jd);

    }

    void Datetime::GetCalendar(double jd, datetime &i_datetime) {
        double s         = 0.0;
        const double mjd = jd - 2400000.5;

        const int J   = static_cast<int>(floor(mjd + 2400000.5 + 0.5));
        const int N   = 4 * (J + 68569) / 146097; //注意这里的除法为整除
        const int L1  = J + 68569 - (N * 146097 + 3) / 4;
        const int Y1  = 4000 * (L1 + 1) / 1461001;
        const int L2  = L1 + 31 - 1461 * Y1 / 4;
        const int M1  = 80 * L2 / 2447;
        const int D   = L2 - 2447 * M1 / 80;
        const int L3  = M1 / 11;
        const int M   = M1 + 2 - 12 * L3;
        const int Y   = 100 * (N - 49) + Y1 + L3;
        s             = (mjd - floor(mjd)) * 86400 + 0.005;
        const int h   = static_cast<int>(floor(s / 3600));
        const int min = static_cast<int>(floor((s - 3600 * h) / 60));
        s             = abs(s - h * 3600 - min * 60 - 0.005);
        s             = tools.round(s * 1e6) / 1e6;

        i_datetime.year = Y;
        i_datetime.month = M;
        i_datetime.day = D;
        i_datetime.hour = h;
        i_datetime.minute = min;
        i_datetime.second = s;
    }

    datetime Datetime::GetCalendar1(double jd) {
        double c;
        datetime i_datetime;
        double seconds = (jd - tools.R2P5(jd)) * 86400;

        const double jd0 = floor(jd + 0.5);
        if (jd0 < 2299161) {
            c = jd0;
        } else {
            const double b = floor(((jd0 - 1867216) - 0.25) / 36524.25);
            c        = jd0 + b - floor(b / 4) + 1;
        }
        c = c + 1524;

        const double d = floor((c - 122.1) / 365.25);
        const double e = 365 * d + floor(d / 4);
        const double f = floor((c - e) / 30.6001);

        i_datetime.month =static_cast<int> (f - 1. - 12. * floor(f / 14.));
        i_datetime.year = static_cast<int> (d - 4715. - floor((7.0 + i_datetime.month) / 10.0));
        i_datetime.day =static_cast<int> ( floor(c - e + 0.5) - floor(30.6001 * f));
        i_datetime.hour = floor(seconds / 3600.);
        seconds = seconds - 3600. * i_datetime.hour;
        i_datetime.minute = floor(seconds / 60.);
        i_datetime.second = seconds - 60. * i_datetime.minute;

        return i_datetime;
    }

    void Datetime::Cal_sun(double *rsun, const double mjd) {
        const double twopi   = 2.0 * M_PI;
        const double deg2rad = M_PI / 180.0;

        const double tut1     = (mjd - 2451545.0) / 36525.0;
        double mean_long = 280.460 + 36000.77 * tut1;

        mean_long = fmod(mean_long, 360.0); //求m/n的余数

        const double t_tdb        = tut1;
        double mean_anomaly = 357.5277233 + 35999.05034 * t_tdb;
        mean_anomaly        = fmod(mean_anomaly * deg2rad, twopi);  //rad
        if (mean_anomaly < 0.0)
            mean_anomaly = twopi + mean_anomaly;

        double ecl_p_long = mean_long + 1.914666471 * sin(mean_anomaly) + 0.019994643 * sin(2.0 * mean_anomaly); //deg
        ecl_p_long        = fmod(ecl_p_long, 360.0);                                                            //deg

        double obliquity = 23.439291 - 0.0130042 * t_tdb;  //deg

        ecl_p_long = ecl_p_long * deg2rad;
        obliquity = obliquity * deg2rad;

        // --------- find magnitude of sun vector, )   components ------
        const double ma_gr = 1.000140612 - 0.016708617 * cos(mean_anomaly) - 0.000139589 * cos(2.0 * mean_anomaly);
        rsun[0]     = ma_gr * cos(ecl_p_long);
        rsun[1]     = ma_gr * cos(obliquity) * sin(ecl_p_long);
        rsun[2]     = ma_gr * sin(obliquity) * sin(ecl_p_long);

    }


    int Datetime::dateDiff(datetime min_date, datetime max_date) {
        int days                   = 0, j, seconds = 0;
        const int primeMonth[][12] = {{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
                                      {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};
        /************************************************************************/
        /*        交换两个日期函数,将小的日期给mindate,将大的日期给maxdate     */
        /************************************************************************/
        if ((min_date.year > max_date.year) || (min_date.year == max_date.year && min_date.month > max_date.month) ||
            (min_date.year == max_date.year && min_date.month == max_date.month && min_date.day > max_date.day)) {
            const datetime tmp = min_date;
            min_date     = max_date;
            max_date     = tmp;
        }
        /************************************************************************/
        /*  从mindate.year开始累加到maxdate.year                                */
        /************************************************************************/
        for (j = min_date.year; j < max_date.year; ++j)
            days += isPrime(j) ? 366 : 365;

        //如果maxdate.year是闰年,则flag=1,后面调用primeMonth[1][12]
        int flag = isPrime(max_date.year);
        //加上maxdate.month到1月的天数
        for (j = 1; j < max_date.month; j++)
            days += primeMonth[flag][j - 1];

        //减去mindate.month到1月的天数

        flag = isPrime(max_date.year);
        for (j = 1; j < min_date.month; j++)
            days -= primeMonth[flag][j - 1];
        days = days + max_date.day - min_date.day;


        //返回时间，单位为秒
        seconds = days * 24 * 60 * 60;
        return seconds;
    }

    datetime Datetime::formatSecond(const double secs) {
        //计算28天以内的秒数
        datetime my_datetime;

        my_datetime.day = static_cast<int>(secs / (60 * 60 * 24));
        if (my_datetime.day < 28) {
            my_datetime.month = 0;
            my_datetime.year = 0;
        } else {
            my_datetime.month = 0;
            my_datetime.year = 0;
        }

        my_datetime.hour   = static_cast<int>(secs / (60 * 60) - my_datetime.day * 24);
        my_datetime.minute = static_cast<int>(secs / 60 - my_datetime.hour * 60 - my_datetime.day * 24 * 60);
        my_datetime.second = static_cast<int>(secs - my_datetime.minute * 60 - my_datetime.hour * 60 * 60 -
                                              my_datetime.day * 24 * 60 * 60);

        return my_datetime;
    }


    void next_second(int &year, int &month, int &day, int &hour, int &minute, int &second) {
        int DayOfMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};  //  每个月的分别对应的天数
        if (second >= 60                                                 // 如果输入不正确的时间会报错，请求你重新输入
            || minute >= 60
            || hour >= 24
            || day > 28 && month == 2 && (0 != year % 4 || 0 == year / 400)
            || day > 29 && month == 2 && (0 == year % 4 && 0 != year / 400)
            || day > 30 && (month == 4 || month == 6 || month == 9 || month == 11)
            || day > 31 && (month == 1 || month == 3 || month == 5 || month == 7)
            || month > 12
            || second <= 0
            || minute <= 0
            || hour <= 0
            || day <= 0
            || month <= 0) {
            //cout << "please input correct time:" << endl;
            year = month = day = hour = minute = second = 0;
            return;    //如果输入不正确的日期时间不正确，返回。
        }
        if (0 == year % 4 && 0 != year / 400)             // 如果是闰年的话，2月份是29天，这里修正了数组。
            DayOfMonth[1] = 29;

        second += 1;
        if (second == 60)                                    // 增加一秒之后，完成日期和时间的更新
        {
            second = 0;
            minute += 1;
            if (minute == 60) {
                minute = 0;
                hour += 1;
                if (hour == 24) {
                    hour = 0;
                    day += 1;
                    if (day > DayOfMonth[month - 1]) {
                        day = 1;
                        month += 1;
                        if (month == 13) {
                            month = 1;
                            year += 1;
                        }
                    }
                }
            }
        }


    }
}
