#pragma once
#include "stdafx.h"
#include "Datetime.h"
#include "Tools.h"
#include "Kepler.h"
#include "OrbitKit.h"
namespace JTC_Basic_OrbitModel {

    class Satellite {

    protected:


    public:
        coe Sat_Status;
        datetime MyEpoch;

        double Rs[3];//Position
        double Vs[3];//Velocity
        double RV[6];//Position and Velocity

        double Mass0;    //卫星初始质量（含燃料质量）
        double Fuel;    //燃料质量
        double EngineF;    //推力
        double Isp;        //比冲
        double Dv;        //速度增量

        void Sat_Position_init(SATELLITE *m_sat, double jd_start, coe sat_coe);

        void Sat_Position(double *RV, double *R, double *V, double jd_start, double jd_end, coe sat_coe);

        void Sat_Position_Pro(double *RV, double *R, double *V, double jd_end, SATELLITE *m_sat);


        void rv_from_r0v0(double *R, double *V, double *R0, double *V0, double t, double mymu);

        //lambert
        void lambert(double *V1, double *V2, double *R1, double *R2, double t, int mystring);

        double circular_T(double r);//圆轨道周期
        double circular_n(double r); //圆轨道速度
        double circular_v(double r);

        double Cal_Dm(double Dv, double Isp, double mass);//估算燃料消耗质量

        double Cal_theta(double *R1, double *R2);

        void Cal_DV(double *Vd_m, double *t0, double *R1, double *R2);

        coe coe_from_sv(double *R, double *V, double mymu);

        coe coe_from_sv1(double *R, double *V, double mymu);

        void sv_from_coe(double *R, double *V, coe mycoe, double mymu);
        //void sv_from_coe(double *R, double *V, double *mycoedd, double mymu);


        ///TLE转六根数函数
        coe TLE2coe(string line1, string line2);

        double TLE2Julian(string line1, string line2);


        ///经度转六根数函数
        coe JW2COE(double lamda, datetime mydatetime, double T0);

        void J2000_WG84(double **A, double &TT, datetime para, double T);

        void greenwich(double **G, double jd);

        void precession(double **P, double jd);

        void nutation(double **N, double jd);

        void rov(double **A, double theta, int flag);

        double range(double x);

        void chektable(double &delta_sigma, double &delta_fai, double T);
        /////////////////

        Tools tools;
        Kepler kepler;

        Satellite(void);

        ~Satellite(void);

    private:


    };

}
