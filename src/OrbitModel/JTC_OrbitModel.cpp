#include "OrbitModel/JTC_OrbitModel.h"
namespace JTC_Basic_OrbitModel {



    JTC_OrbitModel::JTC_OrbitModel(const string& tle_1, const string &tle_2)
    {
        _fileSaver = std::make_shared<FileSaver>("./Results/OrbitModel/");
        _orbitType = SGP4;

        _coe           = _satT1.TLE2coe(tle_1, tle_2);    //计算六根数
        const auto MJD = _satT1.TLE2Julian(tle_1, tle_2); //计算儒略日
        _timeConvertor.GetCalendar(MJD, _epoch);          //转化为年月日

        _sat.modeltype = NEWCREATEDSAT;
        _sat.oe.jd_epoch = MJD;
        _sat.oe.semi_ma = _coe.a;
        _sat.oe.ecc = _coe.e;
        _sat.oe.inc = _coe.i;
        _sat.oe.raan = _coe.o;
        _sat.oe.arg_perigee = _coe.w;
        _sat.oe.mean_M0 = _coe.M;
    }


    /*轨道根数构造*/
    JTC_OrbitModel::JTC_OrbitModel(const int year, const int month, const int day, const int hour, const int minute, const double second,
                                   const double semi_majorAxis, const double eccentricity, const double inclination, const double argOfPerigee, const double rann,
                                   const double meananom)
    {
        _fileSaver = std::make_shared<FileSaver>("./Results/OrbitModel/");
        _orbitType = TwoBody4Classical;
        _epoch = _datetime();
        _epoch.year = year;
        _epoch.month = month;
        _epoch.day = day;
        _epoch.hour = hour;
        _epoch.minute = minute;
        _epoch.second = second;

        _sat.modeltype = NEWCREATEDSAT;
        _sat.oe.jd_epoch = _timeConvertor.MJD(_epoch);
        _sat.oe.semi_ma = semi_majorAxis / 1000.0;
        _sat.oe.ecc = eccentricity;
        _sat.oe.inc = inclination / DEG;
        _sat.oe.raan = rann / DEG;
        _sat.oe.arg_perigee = argOfPerigee / DEG;
        _sat.oe.mean_M0 = meananom / DEG;
    }


    /* 计算给定时刻的位置速度
    返回值（18个数据，地惯系下位置速度，地固系下位置速度，经纬高位置速度）*/
    void JTC_OrbitModel::update(const int year, const int month, const int day, const int hour, const int minute, const double second, const double step)
    {
        _flyTime+=step;
        auto t              = _datetime();
        t.year              = year;
        t.month             = month;
        t.day               = day;
        t.hour              = hour;
        t.minute            = minute;
        t.second            = second;
        const double my_mjd = _timeConvertor.MJD(t);

        switch (_orbitType) {
            case TwoBody4Cartesian://位置速度
            {

            }
                break;
            case TwoBody4Classical://轨道根数
            {
                _orbitKit.SatellitePosition(my_mjd, &_sat);
                _posEci[0] = _sat.sp.x;
                _posEci[1] = _sat.sp.y;
                _posEci[2] = _sat.sp.z;
                _velEci[0] = _sat.sp.vx;
                _velEci[1] = _sat.sp.vy;
                _velEci[2] = _sat.sp.vz;
            }
                break;
            case SGP4://SGP4
            {
//            _timeConvertor.GetCalendar(my_mjd, t);
                _orbitKit.SatellitePosition(my_mjd, &_sat);
                _posEci[0] = _sat.sp.x;
                _posEci[1] = _sat.sp.y;
                _posEci[2] = _sat.sp.z;
                _velEci[0] = _sat.sp.vx;
                _velEci[1] = _sat.sp.vy;
                _velEci[2] = _sat.sp.vz;

            }
                break;
            default:
                break;
        }

        _datetime cal_datetime = _timeConvertor.GetCalendar1(my_mjd);
        double **A = _tools.TwoArrayAlloc(3, 3);
        double TT, T0;
        T0 = 0;
        TT = 0;
        _satT1.J2000_WG84(A, TT, cal_datetime, T0);
        _tools.OnMatDotVec(A, _posEci, _posEcf);
        _tools.OnMatDotVec(A, _velEci, _velEcf);
        CartesianCoordinates cartesian;
        cartesian.x = _posEcf[0] * 1000.;
        cartesian.y = _posEcf[1] * 1000.;
        cartesian.z = _posEcf[2] * 1000.;

        GeodeticCoordinates convertedGeodetic = _tools.XYZtoBLH(cartesian);
        _lla[0] = convertedGeodetic.lon;
        _lla[1] = convertedGeodetic.lat;
        _lla[2] = convertedGeodetic.alt;
        free(A);

        _coe.a = _sat.sp.ma;
        _coe.e = _sat.sp.ecc;
        _coe.i = _sat.sp.inc;
        _coe.o = _sat.sp.raan;
        _coe.w = _sat.sp.u;
        _coe.M = _sat.sp.w;

        _state.posEcf={cartesian.x,cartesian.y,cartesian.z};
        _state.velEcf = {_velEcf[0]*1000.,_velEcf[1]*1000.,_velEcf[2]*1000.};
        _fileSaver->save_traj(this);
    }
}