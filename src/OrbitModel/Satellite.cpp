#include "OrbitModel/Satellite.h"
#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "OrbitModel/stdafx.h"
#include "OrbitModel/Tools.h"
namespace JTC_Basic_OrbitModel {
    using namespace std;

    Satellite::Satellite() {

    }


    Satellite::~Satellite() {

    }

    coe Satellite::TLE2coe(string line1, string line2) {
        coe mycoe;
        /*
	//FILE *fp;
	//char  filedir[2048],tmp[200];
	//strcpy(filedir,tlefile.c_str());

	//double A[]
	//fp = fopen(filedir,"wt");
	//int i=0;
	//while(!feof(fp))
	//{
	//	if(fgets(tmp, 200, fp) == NULL)
	//		break;
	//	i ++;
	//}
	//string name;
	//
	//fscanf(fp,"%s",&name);
	//fscanf(fp,",'%d %d% c%5d%*3c%2d%f%f%5d%*c%*d%5d%*c%*d%d%5d'",&name);

	//sscanf(fp,"%lf %lf %lf %lf %lf %lf %lf ",w_dVar, w_dVar+1, w_dVar+2, w_dVar+3, w_dVar+4, w_dVar+5, w_dVar+6);
	//*/

        /*
	line1:
	19–20	TLE历时（年份后两位）tleYear
	21–32	TLE历时 tleTime

	line2:
	行轨道（从地球北极上空看是逆时针运行）hOrbit
	升交点赤经 o
	轨道偏心率 e
	近地点幅角 w
	平近点角 M
	每天环绕地球的圈数 roundPerDay
	*/
        int tleYear = -1;
        double tleTime, i, n, a, o, e, w, M, roundPerDay;

        //string line1 = "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927";
        //string line2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537";

        string value = line1.substr(18, 2);
        tleYear = atoi(value.c_str());

        value = line1.substr(20, 12);
        tleTime = atof(value.c_str());

        value = line2.substr(7, 9);
        i = atof(value.c_str());

        value = line2.substr(16, 9);
        o = atof(value.c_str());

        value = line2.substr(25, 8);
        e = atof(value.c_str());
        e /= 10000000;

        value = line2.substr(33, 9);
        w = atof(value.c_str());

        value = line2.substr(42, 9);
        M = atof(value.c_str());

        value = line2.substr(51, 12);
        roundPerDay = atof(value.c_str());


        // Get mean motion:
        n = roundPerDay * 2 * PI / 86400;

        a = pow(mu / n / n, 1.0 / 3);

        mycoe.a = a;
        mycoe.e = e;
        mycoe.i = i * PI / 180;
        mycoe.o = o * PI / 180;
        mycoe.w = w * PI / 180;
        mycoe.M = M * PI / 180;


        return mycoe;

    }


    double Satellite::TLE2Julian(string line1, string line2) {
        double JD;
        int tleYear = -1;
        double tleTime;
        string value = line1.substr(18, 2);
        tleYear = atoi(value.c_str());
        tleYear += 2000;

        value = line1.substr(20, 12);
        tleTime = atof(value.c_str());
        tleYear--;

        int A, B;

        A = (long) (tleYear / 100.0);
        B = 2 - A + (long) (A / 4.0);
        JD = (long) (365.25 * tleYear) + (long) (30.6001 * 14) + B + 1720994.5 + tleTime;
        return JD;

    }

    void Satellite::Sat_Position_init(SATELLITE *m_sat, double jd_start, coe sat_coe) {
        m_sat->modeltype = NEWCREATEDSAT;
        m_sat->oe.jd_epoch = jd_start;
        m_sat->oe.semi_ma = sat_coe.a;
        m_sat->oe.ecc = sat_coe.e;
        m_sat->oe.inc = sat_coe.i;
        m_sat->oe.raan = sat_coe.o;
        m_sat->oe.arg_perigee = sat_coe.w;
        m_sat->oe.mean_M0 = sat_coe.M;

    }

    void Satellite::Sat_Position(double *RV, double *R, double *V, double jd_start, double jd_end, coe sat_coe) {
        COrbitKit m_orbitKit;
        SATELLITE m_sat;
        Datetime mydatetime;
        m_sat.modeltype = NEWCREATEDSAT;
        //m_sat.oe.jd_epoch = mydatetime.MJD(start_datetime);
        m_sat.oe.jd_epoch = jd_start;
        m_sat.oe.semi_ma = sat_coe.a;
        m_sat.oe.ecc = sat_coe.e;
        m_sat.oe.inc = sat_coe.i;
        m_sat.oe.raan = sat_coe.o;
        m_sat.oe.arg_perigee = sat_coe.w;
        m_sat.oe.mean_M0 = sat_coe.M;

        //ofstream out("out.txt");
        m_orbitKit.SatellitePosition(jd_end, &m_sat);
        //out << setprecision(15)<<end <<"  "<< setprecision(9) << m_sat.sp.x << "  " << m_sat.sp.y << "  " << m_sat.sp.z << "  "<<m_sat.sp.vx << "  " << m_sat.sp.vy << "  " << m_sat.sp.vz << endl;

        RV[0] = m_sat.sp.x;
        RV[1] = m_sat.sp.y;
        RV[2] = m_sat.sp.z;
        RV[3] = m_sat.sp.vx;
        RV[4] = m_sat.sp.vy;
        RV[5] = m_sat.sp.vz;

        R[0] = m_sat.sp.x;
        R[1] = m_sat.sp.y;
        R[2] = m_sat.sp.z;

        V[0] = m_sat.sp.vx;
        V[1] = m_sat.sp.vy;
        V[2] = m_sat.sp.vz;
    }

    void Satellite::Sat_Position_Pro(double *RV, double *R, double *V, double jd_end, SATELLITE *m_sat) {
        COrbitKit m_orbitKit;
        m_orbitKit.SatellitePosition(jd_end, m_sat);
        //out << setprecision(15)<<end <<"  "<< setprecision(9) << m_sat.sp.x << "  " << m_sat.sp.y << "  " << m_sat.sp.z << "  "<<m_sat.sp.vx << "  " << m_sat.sp.vy << "  " << m_sat.sp.vz << endl;

        RV[0] = m_sat->sp.x;
        RV[1] = m_sat->sp.y;
        RV[2] = m_sat->sp.z;
        RV[3] = m_sat->sp.vx;
        RV[4] = m_sat->sp.vy;
        RV[5] = m_sat->sp.vz;

        R[0] = m_sat->sp.x;
        R[1] = m_sat->sp.y;
        R[2] = m_sat->sp.z;

        V[0] = m_sat->sp.vx;
        V[1] = m_sat->sp.vy;
        V[2] = m_sat->sp.vz;
    }

    void Satellite::rv_from_r0v0(double *R, double *V, double *R0, double *V0, double t, double mymu) {
        double r0, v0, vr0, alpha, f, g, fdot, gdot, x, r;
        double RR1[3], RR2[3], VV1[3], VV2[3];


        r0 = tools.OnNorm(R0, 3);
        v0 = tools.OnNorm(V0, 3);
        vr0 = tools.OnVecDotVec(R0, V0, 3) / r0;
        alpha = 2 / r0 - pow(v0, 2) / mymu;
        x = kepler.Kepler_U(t, r0, vr0, alpha);

        f = kepler.f(x, t, r0, alpha);
        g = kepler.g(x, t, r0, alpha);


        /*R=f*R0+g*V0;*/
        tools.OnVecDotNum(R0, f, RR1);
        tools.OnVecDotNum(V0, g, RR2);
        tools.OnVecAddVec(RR1, RR2, R, 3);


        r = tools.OnNorm(R, 3);
        fdot = kepler.fdot(x, r, r0, alpha);
        gdot = kepler.gdot(x, r, r0, alpha);
        /*V=fdot*R0+gdot*V0;*/
        tools.OnVecDotNum(R0, fdot, VV1);
        tools.OnVecDotNum(V0, gdot, VV2);
        tools.OnVecAddVec(VV1, VV2, V, 3);

    }

    double Satellite::Cal_theta(double *R1, double *R2) {//计算两个卫星的夹角
        double r1, r2, theta;
        r1 = tools.OnNorm(R1, 3);
        r2 = tools.OnNorm(R2, 3);
        theta = acos(tools.OnVecDotVec(R1, R2, 3) / r1 / r2);

        return theta;
    }

    void Satellite::lambert(double *V1, double *V2, double *R1, double *R2, double t, int mystring) {
        double r1, r2, theta, A, z, tol, ratio;
        double cl2[3];
        int n, nmax;

        r1 = tools.OnNorm(R1, 3);
        r2 = tools.OnNorm(R2, 3);

        tools.OnVecByVec(R1, R2, cl2);
        theta = acos(tools.OnVecDotVec(R1, R2, 3) / r1 / r2);

        if (mystring >= 1)
            if (cl2[2] <= 0) {
                theta = 2 * PI - theta;
            }
        if (mystring < 1)
            if (cl2[2] >= 0) {
                theta = 2 * PI - theta;
            }

        A = sin(theta) * sqrt(r1 * r2 / (1 - cos(theta)));
        z = -100;
        while (kepler.F(r1, r2, A, z, t) < 0) {
            z = z + 0.1;
        }
        tol = 1.e-8;
        nmax = 5000;
        ratio = 1;
        n = 0;

        while ((abs(ratio) > tol) & (n <= nmax)) {
            n = n + 1;
            ratio = kepler.F(r1, r2, A, z, t) / kepler.dFdz(r1, r2, A, z, t);
            z = z - ratio;
        }
        double f, g, gdot;

        f = 1 - kepler.y(r1, r2, A, z) / r1;
        g = A * sqrt(kepler.y(r1, r2, A, z) / mu);
        gdot = 1 - kepler.y(r1, r2, A, z) / r2;

        double fR1[3], gdotR2[3], V11[3], V21[3];

        tools.OnVecDotNum(R1, f, fR1);
        tools.OnVecSubVec(R2, fR1, V11, 3);
        tools.OnVecDotNum(V11, 1 / g, V1);

        /*V1   = 1/g*(R2 - f*R1);
	V2   = 1/g*(gdot*R2 - R1);*/
        tools.OnVecDotNum(R2, gdot, gdotR2);
        tools.OnVecSubVec(gdotR2, R1, V21, 3);
        tools.OnVecDotNum(V21, 1 / g, V2);

    }


    double Satellite::circular_T(double r)//圆轨道周期
    {   //r单位为km
        double T;
        T = 2 * PI / sqrt(mu / (pow(r, 3)));
        return T;
    }


    double Satellite::circular_n(double r) //圆轨道角速度w
    {   //r单位为km
        double n;
        n = sqrt(mu / (pow(r, 3)));
        return n;
    }

    double Satellite::circular_v(double r) //漂移速率km/s
    {   //r单位为km
        double n;
        n = sqrt(mu / (pow(r, 3))) * r;
        return n;
    }

    double Satellite::Cal_Dm(double Dv, double Isp, double Mass) {
        return Mass * abs(Dv) / Isp / g_earth;

        /*dm1=(1-pow(Ee,-tools.OnNorm(dv11,3)*1000/(mySat1.Isp*g_earth)))*mySat1.Mass0;*/
    }


    void Satellite::Cal_DV(double *Vd_m, double *tt, double *R1, double *R2) {
        double t0, kesi, miu, R_E, g0;

        t0 = 0;//初始时刻
        kesi = 0.001;//时间计算精度
        miu = 3.986005e14;
        R_E = 6371004;
        g0 = 9.8;


        double R_pip, R_m, theta_f, V0, lamda;
        double gamma_min, gamma_max, gamma0;

        R_pip = sqrt(tools.OnVecDotVec(R2, R2, 3));
        R_m = sqrt(tools.OnVecDotVec(R1, R1, 3));
        theta_f = acos(tools.OnVecDotVec(R1, R2, 3) / R_m / R_pip);


        //弹道倾角范围：
        gamma_min = atan((cos(theta_f) - R_m / R_pip) / sin(theta_f));
        gamma_max = atan((sin(theta_f) + sqrt((1 - cos(theta_f)) * 2 * R_m / R_pip)) / (1 - cos(theta_f)));

        //迭代求解
        gamma0 = (gamma_min + gamma_max) / 2.0;
        V0 = R_pip * (1.0 - cos(theta_f)) * miu / R_m /
             (R_m * pow(cos(gamma0), 2) - R_pip * cos(theta_f + gamma0) * cos(gamma0));
        V0 = sqrt(V0);
        lamda = R_m * pow(V0, 2) / miu;
        if ((lamda > 0) && (lamda < 2)) {
            t0 = (tan(gamma0) * (1.0 - cos(theta_f)) + (1.0 - lamda) * sin(theta_f)) / (2.0 - lamda) / R_m * R_pip;
            t0 = t0 + 2.0 * cos(gamma0) *
                      atan(sqrt(2.0 / lamda - 1.0) / (cos(gamma0) * (1.0 / tan(theta_f / 2.0)) - sin(gamma0))) / lamda /
                      pow(sqrt(2.0 / lamda - 1.0), 3);
            t0 = t0 * R_m / V0 / cos(gamma0);
        } else if (lamda > 2) {
            t0 = (tan(gamma0) * (1.0 - cos(theta_f)) + (1.0 - lamda) * sin(theta_f)) / (2.0 - lamda) / R_m * R_pip;
            t0 = t0 - cos(gamma0) *
                      log((sin(lamda) - cos(lamda) * (1.0 / tan(theta_f / 2.0)) - sqrt(1.0 - 2.0 / lamda)) /
                          (sin(lamda) - cos(lamda) * (1.0 / tan(theta_f / 2.0)) + sqrt(1.0 - 2.0 / lamda)));
            t0 = t0 * R_m / V0 / cos(gamma0);

        } else if (lamda == 2) {
            t0 = 3 * cos(lamda) * (1 / tan(theta_f / 2.0)) / pow(cos(lamda) * (1 / tan(theta_f / 2.0)) - sin(lamda), 2);
            t0 = 1 / pow(cos(lamda) * (1 / tan(theta_f / 2.0)) - sin(lamda), 3);
            t0 = t0 * 2 * R_m / (3 * V0);
        }

        //期望的速度：
        double i_vec[3], j_vec[3], V11[3], V12[3], V21[3], V22[3], temp;

        tools.OnVecDotNum(R1, 1 / R_m, i_vec);
        tools.OnVecByVec(R1, R2, V11);
        tools.OnVecByVec(V11, R1, V12);
        temp = tools.OnVecDotVec(V12, V12, 3);
        tools.OnVecDotNum(R1, 1 / temp, j_vec);

        tools.OnVecDotNum(i_vec, V0 * sin(gamma0), V21);
        tools.OnVecDotNum(j_vec, V0 * cos(gamma0), V22);
        tools.OnVecAddVec(V21, V22, Vd_m, 3);
        tt[0] = t0;


        ////最小
        //gamma0=gamma_min;
        //V0=R_pip*(1-cos(theta_f))*miu/R_m/(R_m*pow(cos(gamma0),2)-R_pip*cos(theta_f+gamma0)*cos(gamma0));
        //V0=sqrt(V0);
        //lamda=R_m*pow(V0,2)/miu;
        //if((lamda>0)&&(lamda<2))
        //{
        //	t0=(tan(gamma0)*(1.0-cos(theta_f))+(1.0-lamda)*sin(theta_f))/(2.0-lamda)/R_m*R_pip;
        //	t0=t0+2.0*cos(gamma0)*atan(sqrt(2.0/lamda-1.0)/(cos(gamma0)*(1.0/tan(theta_f/2.0))-sin(gamma0)))/lamda/pow(sqrt(2.0/lamda-1.0),3);
        //	t0=t0*R_m/V0/cos(gamma0);
        //}
        //else if(lamda>2)
        //{
        //	double temp;
        //	t0=(tan(gamma0)*(1.0-cos(theta_f))+(1.0-lamda)*sin(theta_f))/(2.0-lamda)/R_m*R_pip;
        //	temp=log( (sin(lamda)-cos(lamda)*(1.0/tan(theta_f/2.0))-sqrt(1.0-2.0/lamda))/(sin(lamda)-cos(lamda)*(1.0/tan(theta_f/2.0))+sqrt(1.0-2.0/lamda)) );
        //	t0=t0-cos(lamda)*temp/(lamda*pow(sqrt(lamda/(1-2/lamda)),3));
        //	t0=t0*R_m/V0/cos(gamma0);

        //}
        //else if(lamda==2)
        //{
        //	t0=3*cos(lamda)*(1/tan(theta_f/2.0))/pow(cos(lamda)*(1/tan(theta_f/2.0))-sin(lamda),2);
        //	t0=1/pow(cos(lamda)*(1/tan(theta_f/2.0))-sin(lamda),3);
        //	t0=t0*2*R_m/(3*V0);
        //}
        //tt[1]=t0;

        ////最大
        //gamma0=gamma_max;
        //V0=R_pip*(1-cos(theta_f))*miu/R_m/(R_m*pow(cos(gamma0),2)-R_pip*cos(theta_f+gamma0)*cos(gamma0));
        //V0=sqrt(V0);
        //lamda=R_m*pow(V0,2)/miu;
        //if((lamda>0)&&(lamda<2))
        //{
        //	t0=(tan(gamma0)*(1.0-cos(theta_f))+(1.0-lamda)*sin(theta_f))/(2.0-lamda)/R_m*R_pip;
        //	t0=t0+2.0*cos(gamma0)*atan(sqrt(2.0/lamda-1.0)/(cos(gamma0)*(1.0/tan(theta_f/2.0))-sin(gamma0)))/lamda/pow(sqrt(2.0/lamda-1.0),3);
        //	t0=t0*R_m/V0/cos(gamma0);
        //}
        //else if(lamda>2)
        //{
        //	t0=(tan(gamma0)*(1.0-cos(theta_f))+(1.0-lamda)*sin(theta_f))/(2.0-lamda)/R_m*R_pip;
        //	t0=t0-cos(gamma0)*log( (sin(lamda)-cos(lamda)*(1.0/tan(theta_f/2.0))-sqrt(1.0-2.0/lamda))/(sin(lamda)-cos(lamda)*(1.0/tan(theta_f/2.0))+sqrt(1.0-2.0/lamda)) );
        //	t0=t0*R_m/V0/cos(gamma0);

        //}
        //else if(lamda==2)
        //{
        //	t0=3*cos(lamda)*(1/tan(theta_f/2.0))/pow(cos(lamda)*(1/tan(theta_f/2.0))-sin(lamda),2);
        //	t0=1/pow(cos(lamda)*(1/tan(theta_f/2.0))-sin(lamda),3);
        //	t0=t0*2*R_m/(3*V0);
        //}
        //tt[2]=t0;



    }

    coe Satellite::coe_from_sv1(double *R, double *V, double mymu) {
        double myeps, r, v, vr, h, n;
        double H[3], N[3];
        double vec[3] = {0, 0, 1};
        coe calcoe;
        double PC;
        PC = 0.99999999999;

        myeps = 1.e-6;

        v = tools.OnNorm(V, 3);
        r = tools.OnNorm(R, 3);
        vr = tools.OnVecDotVec(R, V, 3) / r;

        tools.OnVecByVec(R, V, H);
        h = tools.OnNorm(H, 3);

        calcoe.i = acos(H[2] / h);

        tools.OnVecByVec(vec, H, N);
        n = tools.OnNorm(N, 3);

        if (calcoe.i != 0) {
            calcoe.o = acos(N[0] / n);
            if (N[1] < 0) {
                calcoe.o = 2 * PI - calcoe.o;
            }
        } else {
            calcoe.o = 0;
        }


        //E = 1/mymu*((pow(v,2) - mymu/r)*R - r*vr*V);
        double dr1, dr2;
        double ER[3], EV[3], E1[3], E[3];
        dr1 = pow(v, 2) - mymu / r;
        tools.OnVecDotNum(R, dr1, ER);

        dr2 = r * vr;
        tools.OnVecDotNum(V, dr2, EV);
        tools.OnVecSubVec(ER, EV, E1, 3);
        tools.OnVecDotNum(E1, 1 / mymu, E);

        calcoe.e = tools.OnNorm(E, 3);


        if (calcoe.i != 0) {
            if (calcoe.e > myeps) {
                calcoe.w = acos(tools.OnVecDotVec(N, E, 3) / n / calcoe.e);
                if (E[2] < 0) {
                    calcoe.w = 2 * PI - calcoe.w;
                }
            } else {
                calcoe.w = 0;
            }
        } else {
            if (calcoe.e > myeps) {

                tools.OnVecDotNum(V, dr2, EV);
                calcoe.w = acos(E[0] / calcoe.e);
                if (E[1] < 0) {
                    calcoe.w = 2 * PI - calcoe.w;
                }
            } else {
                calcoe.w = 0;
            }
        }




        ///*

        if (calcoe.i != 0) {
            if (calcoe.e > myeps) {

                //calcoe.M=acos(tools.OnVecDotVec(E,R,3)/calcoe.e/r);
                //if (abs(tools.OnVecDotVec(E,R,3)/calcoe.e/r)>=1)
                //{
                //	calcoe.M=acos(1);
                //}
                //else
                //{
                //	calcoe.M=acos(tools.OnVecDotVec(E,R,3)/calcoe.e/r);
                //}
                calcoe.M = acos(tools.OnVecDotVec(E, R, 3) / calcoe.e / r * PC);
                if (vr < 0) {
                    calcoe.M = 2 * PI - calcoe.M;
                }
            } else {
                //calcoe.M=acos(tools.OnVecDotVec(N,R,3)/n/r);
                calcoe.M = acos(tools.OnVecDotVec(N, R, 3) / n / r * PC);
                if (R[2] < 0) {
                    calcoe.M = 2 * PI - calcoe.M;
                }
            }
        } else {
            if (calcoe.e < myeps) {
                //calcoe.M=acos(tools.OnVecDotVec(E,R,3)/calcoe.e/r);
                calcoe.M = acos(tools.OnVecDotVec(E, R, 3) / calcoe.e / r * PC);
                if (vr < 0) {
                    calcoe.M = 2 * PI - calcoe.M;
                }
            } else {
                //calcoe.M=acos(R[0]/r);
                calcoe.M = acos(R[0] / r * PC);
                if (R[1] < 0) {
                    calcoe.M = 2 * PI - calcoe.M;
                }
            }

        }
        //*////////////

        /*//
	if ( calcoe.i != 0)
	{
		if (calcoe.e > myeps)
		{

			calcoe.M=acos(tools.OnVecDotVec(E,R,3)/calcoe.e/r);
			//calcoe.M=acos(tools.OnVecDotVec(E,R,3)/calcoe.e/r);
			if (vr<0)
			{
				calcoe.M=2*PI-calcoe.M;
			}
		}
		else
		{
			calcoe.M=acos(tools.OnVecDotVec(N,R,3)/n/r);
			//calcoe.M=acos(tools.OnVecDotVec(N,R,3)/n/r*PC);
			if (R[2]<0)
			{
				calcoe.M=2*PI-calcoe.M;
			}
		}
	}
	else
	{
		if (calcoe.e < myeps)
		{
			calcoe.M=acos(tools.OnVecDotVec(E,R,3)/calcoe.e/r);
			//calcoe.M=acos(tools.OnVecDotVec(E,R,3)/calcoe.e/r*PC);
			if (vr<0)
			{
				calcoe.M=2*PI-calcoe.M;
			}
		}
		else
		{
			calcoe.M=acos(R[0]/r);
			//calcoe.M=acos(R[0]/r*PC);
			if (R[1]<0)
			{
				calcoe.M=2*PI-calcoe.M;
			}
		}

	}
	///*/

        calcoe.a = pow(h, 2) / mu / (1 - pow(calcoe.e, 2));

        return calcoe;

    }

    coe Satellite::coe_from_sv(double *R, double *V, double mymu) {
        //double myeps,vr,h,RK,OMG,omg,e,ndotr,sat,E,M,u;
        double r[3], v[3];
        for (int ii = 0; ii < 3; ii++) {
            r[ii] = R[ii];
            v[ii] = V[ii];
        }

        double rv[3], N[3], n[3], V_rv[3], v_rv[3], rr[3], ec[3], ee[3];
        double h, RK, i, OMG, omg, ndotr, sat, E, M, u, a, e, mymu0;

        double vec[3] = {0, 0, 1};
        coe calcoe;


        tools.OnVecByVec(r, v, rv);
        h = tools.OnNorm(rv, 3);
        RK = 180 / PI;
        i = acos(rv[2] / h);//rad
        mymu0 = 3.986004e5;
        a = 0.5 * mymu0 / (mymu0 / tools.OnNorm(r, 3) - 0.5 * tools.OnNorm(v, 3) * tools.OnNorm(v, 3));

        //————模型修正————//
        if (i == 0 || i == PI) {
            v[2] = v[2] + 1e-10;
        }
        tools.OnVecByVec(r, v, rv);
        //————模型修正结束————//

        tools.OnVecByVec(vec, rv, N);
        tools.OnVecDotNum(N, 1 / tools.OnNorm(N, 3), n);

        OMG = acos(n[0]) * RK;

        if (n[1] < 0) {
            OMG = 360 - acos(n[0]) * RK;
        }
        tools.OnVecByVec(V, rv, V_rv);
        tools.OnVecDotNum(V_rv, 1 / mymu0, v_rv);
        tools.OnVecDotNum(R, 1 / tools.OnNorm(R, 3), rr);
        tools.OnVecSubVec(v_rv, rr, ec, 3);
        e = tools.OnNorm(ec, 3);

        if (e < 1e-10) {
            omg = 0;
        }
        if (e >= 1e-10) {
            tools.OnVecDotNum(ec, 1 / tools.OnNorm(ec, 3), ee);

        }
        omg = acos(tools.OnVecDotVec(ee, n, 3) * 0.99999999999999) * RK;

        if (ee[2] < 0) {
            omg = 360 - omg;
        }
        ndotr = tools.OnVecDotVec(n, r, 3) / tools.OnNorm(n, 3) / tools.OnNorm(r, 3);
        if (r[2] >= 0) {
            u = acos(ndotr * 0.9999999999999999) * RK;
        } else {
            u = 360 - acos(ndotr * 0.9999999999999999) * RK;
        }
        sat = (u - omg) / RK;
        E = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(sat / 2));
        M = RK * (E - e * sin(E));
        if (M < 0) {
            M = M + 360;
        }
        if (M > 360) {
            M = M - 360;
        }
        calcoe.a = a;
        calcoe.i = i;
        calcoe.e = e;
        calcoe.o = OMG * PI / 180;
        calcoe.w = omg * PI / 180;
        calcoe.M = M * PI / 180;

        return calcoe;
    }

    void Satellite::sv_from_coe(double *R, double *V, coe mycoe, double mymu) {
        double h;
        double rp[3], vp[3];
        double onex[3] = {1.0, 0.0, 0.0};
        double oney[3] = {0.0, 1.0, 0.0};
        double nx[3], ny[3], Ret[3];
        double dd;
        double **R3_W = tools.TwoArrayAlloc(3, 3);
        double **R1_i = tools.TwoArrayAlloc(3, 3);
        double **R3_w1 = tools.TwoArrayAlloc(3, 3);
        double **M3 = tools.TwoArrayAlloc(3, 3);
        double **Q_pX = tools.TwoArrayAlloc(3, 3);
        double **Q_pXT = tools.TwoArrayAlloc(3, 3);


        h = sqrt((mycoe.a) * (1 - pow((mycoe.e), 2)) * mymu);

        dd = cos(mycoe.M);
        tools.OnVecDotNum(onex, dd, nx);
        dd = sin(mycoe.M);
        tools.OnVecDotNum(oney, dd, ny);
        tools.OnVecAddVec(nx, ny, Ret, 3);
        dd = (pow(h, 2) / mymu) * (1 / (1 + mycoe.e * cos(mycoe.M)));
        tools.OnVecDotNum(Ret, dd, rp);

        dd = -sin(mycoe.M);
        tools.OnVecDotNum(onex, dd, nx);
        dd = (mycoe.e + cos(mycoe.M));
        tools.OnVecDotNum(oney, dd, ny);
        tools.OnVecAddVec(nx, ny, Ret, 3);
        tools.OnVecDotNum(Ret, mymu / h, vp);


        R3_W[0][0] = cos(mycoe.o);
        R3_W[0][1] = sin(mycoe.o);
        R3_W[0][2] = 0.0;
        R3_W[1][0] = -sin(mycoe.o);
        R3_W[1][1] = cos(mycoe.o);
        R3_W[1][2] = 0.0;
        R3_W[2][0] = 0.0;
        R3_W[2][1] = 0.0;
        R3_W[2][2] = 1.0;

        R1_i[0][0] = 1.0;
        R1_i[0][1] = 0.0;
        R1_i[0][2] = 0.0;
        R1_i[1][0] = 0.0;
        R1_i[1][1] = cos(mycoe.i);
        R1_i[1][2] = sin(mycoe.i);
        R1_i[2][0] = 0.0;
        R1_i[2][1] = -sin(mycoe.i);
        R1_i[2][2] = cos(mycoe.i);

        R3_w1[0][0] = cos(mycoe.w);
        R3_w1[0][1] = sin(mycoe.w);
        R3_w1[0][2] = 0.0;
        R3_w1[1][0] = -sin(mycoe.w);
        R3_w1[1][1] = cos(mycoe.w);
        R3_w1[1][2] = 0.0;
        R3_w1[2][0] = 0.0;
        R3_w1[2][1] = 0.0;
        R3_w1[2][2] = 1.0;


        tools.OnMatByMat(R3_w1, R1_i, M3);
        tools.OnMatByMat(M3, R3_W, Q_pX);
        tools.OnMatTranspose(Q_pX, Q_pXT);
        tools.OnMatDotVec(Q_pXT, rp, R);
        tools.OnMatDotVec(Q_pXT, vp, V);


        //TwoArrayFree(R3_W);
        //TwoArrayFree(R1_i);
        //TwoArrayFree(R3_w1);
        //TwoArrayFree(Q_pX);
        //TwoArrayFree(Q_pXT);
        //TwoArrayFree(M3);


    }

    coe Satellite::JW2COE(double lamda, datetime mydatetime, double T0) {

        double lamda0, miu, S_a, S_T, S_v, S_i, S_h;
        lamda0 = lamda * PI / 180;
        miu = 3.986004e5;
        S_a = 42166.258681;
        S_T = 2 * PI / sqrt(pow(S_a, 3) / miu);
        S_v = sqrt(miu / S_a);
        S_i = 0.118885 * PI / 180;
        S_h = S_a * S_v;

        double r0[3], rr0[3], a1, b1, c1[2], result[2];
        r0[0] = S_a * cos(lamda);
        r0[1] = S_a * sin(lamda);
        r0[2] = 0;

        double **A = tools.TwoArrayAlloc(3, 3);
        double **AT = tools.TwoArrayAlloc(3, 3);
        double JH[2][2];
        double **JH_t = tools.TwoArrayAlloc(2, 2);
        double TT;
        TT = 0;

        J2000_WG84(A, TT, mydatetime, T0);

        //printf("\n A: \n");
        //for (int i=0;i<3;i++)
        //{
        //	for (int j=0;j<3;j++)
        //	{
        //		printf("%f   ",A[i][j]);
        //	}
        //	printf("\n");
        //}

        tools.OnMatTranspose(A, AT);
        printf("\n AT: \n");
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                printf("%f   ", AT[i][j]);
            }
            printf("\n");
        }

        tools.OnMatDotVec(AT, r0, rr0);

        a1 = rr0[0];
        b1 = rr0[1];

        double x, y, z;

        if (a1 != 0 && b1 != 0) {
            JH[0][0] = a1;
            JH[0][1] = b1;
            JH[1][0] = -b1;
            JH[1][1] = a1;

            tools.MatrixInv(JH[0], 2);

            printf("\n JH: \n");
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                    printf("%16.10f   ", JH[i][j]);
                }
                printf("\n");
            }
            c1[0] = 0;
            c1[1] = S_h * cos(S_i);

            JH_t[0][0] = JH[0][0];
            JH_t[0][1] = JH[0][1];
            JH_t[1][0] = JH[1][0];
            JH_t[1][1] = JH[1][1];

            //tools.OnMatDotVec(JH_t,c1,result);
            double xw[2];
            for (int i = 0; i < 2; i++) {
                xw[i] = c1[i];
            }

            for (int i = 0; i < 2; i++) {
                result[i] = 0;
                for (int j = 0; j < 2; j++)
                    result[i] += JH_t[i][j] * xw[j];
            }

            x = result[0];
            y = result[1];
            z = sqrt(S_v * S_v - x * x - y * y);

        } else if (a1 == 0) {
            x = -S_h * cos(S_i) / b1;
            y = 0;
            z = sqrt(S_v * S_v - x * x);//自西向东，否则加负号
        } else if (b1 == 0) {
            x = 0;
            y = S_h * cos(S_i) / a1;
            z = sqrt(S_v * S_v + y * y);//自西向东，否则加负号
        }
        double vv0[3];
        vv0[0] = x;
        vv0[1] = y;
        vv0[2] = z;

        return coe_from_sv(rr0, vv0, miu);


    }


    void Satellite::J2000_WG84(double **A, double &TT, datetime para, double T) {
        int year, month, day, hour, minute;
        double second;

        year = para.year;
        month = para.month;
        day = para.day;
        hour = para.hour;
        minute = para.minute;
        second = para.second + T;

        datetime para1;
        Datetime mydatetime;
        para1.year = year;
        para1.month = month;
        para1.day = day;
        para1.hour = hour;
        para1.minute = minute;
        para1.second = second;

        double JD_0, Julian_Data;
        JD_0 = mydatetime.MJD(para1);
        Julian_Data = JD_0;

        //A=greenwich(Julian_Data)*nutation(Julian_Data)*precession(Julian_Data);
        double **P = tools.TwoArrayAlloc(3, 3);
        double **N = tools.TwoArrayAlloc(3, 3);
        double **G = tools.TwoArrayAlloc(3, 3);
        double **temp = tools.TwoArrayAlloc(3, 3);

        precession(P, Julian_Data);
        nutation(N, Julian_Data);
        greenwich(G, Julian_Data);
        //printf("\n G: \n");
        //for (int i=0;i<3;i++)
        //{
        //	for (int j=0;j<3;j++)
        //	{
        //		printf("%f   ",G[i][j]);
        //	}
        //	printf("\n");
        //}
        //printf("\n N: \n");
        //for (int i=0;i<3;i++)
        //{
        //	for (int j=0;j<3;j++)
        //	{
        //		printf("%f   ",N[i][j]);
        //	}
        //	printf("\n");
        //}
        //printf("\n P: \n");
        //for (int i=0;i<3;i++)
        //{
        //	for (int j=0;j<3;j++)
        //	{
        //		printf("%f   ",P[i][j]);
        //	}
        //	printf("\n");
        //}

        tools.OnMatByMat(N, P, temp);
        tools.OnMatByMat(G, temp, A);
        TT = Julian_Data;

    }

    void Satellite::greenwich(double **G, double jd) {
        double deg;
        deg = PI / 180;

        double T, es, GMT, theta;
        T = (jd - 2451545.0) / 36525.0;
        es = 23.0 + 26.0 / 60.0 + 21.448 / 3600 - 41.8150 / 3600 * T - 0.000597 / 3600 * pow(T, 2) -
             0.00181 / 3600 * pow(T, 3);
        double delta_sigma, delta_fai;
        chektable(delta_sigma, delta_fai, T);
        GMT = 67310.54841 + (8640184.812866 + 876600.0 * 3600.0) * T + 0.093104 * pow(T, 2) - (6.2e-6) * pow(T, 3) +
              delta_fai / 15.0 * 3600.0 * cos((es + delta_sigma) * deg);
        theta = GMT / 3600.0 * 15.0;
        theta = range(theta);
        rov(G, theta, 3);
    }

    void Satellite::precession(double **P, double jd) {
        double T, x1, x2, x3;
        T = (jd - 2451545) / 36525;
        x1 = (2306.2181 * T + 0.30188 * pow(T, 2) + 0.017998 * pow(T, 3)) / 3600;
        x2 = (2004.3109 * T - 0.42665 * pow(T, 2) - 0.041833 * pow(T, 3)) / 3600;
        x3 = (2306.2181 * T + 1.09468 * pow(T, 2) + 0.018203 * pow(T, 3)) / 3600;
        double **rov_z1 = tools.TwoArrayAlloc(3, 3);
        double **rov_y = tools.TwoArrayAlloc(3, 3);
        double **rov_z2 = tools.TwoArrayAlloc(3, 3);
        double **temp = tools.TwoArrayAlloc(3, 3);
        rov(rov_z1, -x3, 3);
        rov(rov_y, x2, 2);
        rov(rov_z2, -x1, 3);

        tools.OnMatByMat(rov_y, rov_z2, temp);
        tools.OnMatByMat(rov_z1, temp, P);


    }

    void Satellite::nutation(double **N, double jd) {
        double T, es, x2, x3;
        T = (jd - 2451545) / 36525;
        es = 23 + 26 / 60 + 21.448 / 3600 - 41.8150 / 3600 * T - 0.000597 * pow(T, 2) - 0.00181 * pow(T, 3);
        double delta_sigma, delta_fai;
        chektable(delta_sigma, delta_fai, T);

        double **rov_x1 = tools.TwoArrayAlloc(3, 3);
        double **rov_x2 = tools.TwoArrayAlloc(3, 3);
        double **rov_z = tools.TwoArrayAlloc(3, 3);
        double **temp = tools.TwoArrayAlloc(3, 3);
        rov(rov_x1, -es - delta_sigma, 1);
        rov(rov_z, -delta_fai, 2);
        rov(rov_x2, es, 1);

        tools.OnMatByMat(rov_z, rov_x2, temp);
        tools.OnMatByMat(rov_x1, temp, N);

    }

    void Satellite::rov(double **A, double theta, int flag) {
        double mytheta;
        mytheta = theta * PI / 180;
        //switch(flag)
        //{
        //	case 1:
        //		//A = [1 0 0;0 cos(theta) sin(theta);0 -sin(theta) cos(theta)];
        //		A[0][0] = 1;	A[0][1] = 0;		    A[0][2] = 0;
        //		A[1][0] = 0;	A[1][1] = sin(theta);	A[1][2] = sin(theta);
        //		A[2][0] = 0;	A[2][1] =-sin(theta);	A[2][2] = cos(theta);
        //		break;
        //	case 2:
        //		 //A = [cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
        //		A[0][0] = cos(theta);	A[0][1] = 0;		    A[0][2] = -sin(theta);
        //		A[1][0] = 0;			A[1][1] = 1;			A[1][2] = 0;
        //		A[2][0] = sin(theta);	A[2][1] =0;				A[2][2] = cos(theta);
        //		break;
        //	case 3:
        //		//A = [cos(theta) sin(theta) 0;-sin(theta) cos(theta) 0;0 0 1];
        //		A[0][0] = cos(theta);	A[0][1] = sin(theta);	A[0][2] = 0;
        //		A[1][0] = -sin(theta);	A[1][1] = cos(theta);	A[1][2] = 0;
        //		A[2][0] = 0;			A[2][1] =0;				A[2][2] = 1;
        //		break;

        //}
        if (flag == 1) {
            tools.OnMatX(A, mytheta);
        }
        if (flag == 2) {
            tools.OnMatY(A, mytheta);
        }
        if (flag == 3) {
            tools.OnMatZ(A, mytheta);
        }

        double temp_array1[3], temp_array2[3], temp_array3[3];
        double array1[3], array2[3], array3[3];
        for (int i = 0; i < 3; i++) {
            temp_array1[i] = A[i][0];
            temp_array2[i] = A[i][1];
            temp_array3[i] = A[i][2];
        }
        tools.OnVecDotNum(temp_array1, tools.OnNorm(temp_array1, 3), array1);
        tools.OnVecDotNum(temp_array2, tools.OnNorm(temp_array2, 3), array2);
        tools.OnVecDotNum(temp_array3, tools.OnNorm(temp_array3, 3), array3);
        for (int i = 0; i < 3; i++) {
            A[i][0] = array1[i];
            A[i][1] = array2[i];
            A[i][2] = array3[i];
        }


    }

    double Satellite::range(double x) {
        if (x >= 360) {
            x = x - floor(x / 360.) * 360.;
        } else {
            x = x - (floor(x / 360.) - 1) * 360.;
        }
        return x;
    }

    void Satellite::chektable(double &delta_sigma, double &delta_fai, double T) {
        double sigma, fai;
        double aa[106][10] = {
                {1,   0,     0,  0,  0,  1,   -171996, -174.2, 92025, 8.9},
                {2,   0,     0,  0,  0,  2,   2062,    0.2,    -895,  0.5},
                {3,   -2,    0,  2,  0,  1,   46,      0,      -24,   0},
                {4,   2,     0,  -2, 0,  0,   11,      0,      0,     0},
                {5,   -2,    0,  2,  0,  2,   -3,      0,      1,     0},
                {6,   1,     -1, 0,  -1, 0,   -3,      0,      0,     0},
                {7,   0,     -2, 2,  -2, 1,   -2,      0,      1,     0},
                {8,   2,     0,  -2, 0,  1,   1,       0,      0,     0},
                {9,   0,     0,  2,  -2, 2,   -13187,  -1.6,   5736,  -3.1},
                {10,  0,     1,  0,  0,  0,   1426,    -3.4,   54,    -0.1},
                {11,  0,     1,  2,  -2, 2,   -517,    1.2,    224,   -0.6},
                {12,  0,     -1, 2,  -2, 2,   217,     -0.5,   -95,   0.3},
                {13,  0,     0,  2,  -2, 1,   129,     0.1,    -70,   0},
                {14,  2,     0,  0,  -2, 0,   48,      0,      1,     0},
                {15,  0,     0,  2,  -2, 0,   -22,     0,      0,     0},
                {16,  0,     2,  0,  0,  0,   17,      -0.1,   0,     0},
                {17,  0,     1,  0,  0,  1,   -15,     0,      9,     0},
                {18,  0,     2,  2,  -2, 2,   -16,     0.1,    7,     0},
                {19,  0 - 1, 0,  0,  1,  -12, 0,       6,      0},
                {20,  -2,    0,  0,  2,  1,   -6,      0,      3,     0},
                {21,  0,     -1, 2,  -2, 1,   -5,      0,      3,     0},
                {22,  2,     0,  0,  -2, 1,   4,       0,      -2,    0},
                {23,  0,     1,  2,  -2, 1,   4,       0,      -2,    0},
                {24,  1,     0,  0,  -1, 0,   -4,      0,      0,     0},
                {25,  2,     1,  0,  -2, 0,   1,       0,      0,     0},
                {26,  0,     0,  -2, 2,  1,   1,       0,      0,     0},
                {27,  0,     1,  -2, 2,  0,   -1,      0,      0,     0},
                {28,  0,     1,  0,  0,  2,   1,       0,      0,     0},
                {29,  -1,    0,  0,  1,  1,   1,       0,      0,     0},
                {30,  0,     1,  2,  -2, 0,   -1,      0,      0,     0},
                {31,  0,     0,  2,  0,  2,   -2274,   -0.2,   977,   -0.5},
                {32,  1,     0,  0,  0,  0,   712,     0.1,    -7,    0},
                {33,  0,     0,  2,  0,  1,   -386,    -0.4,   200,   0},
                {34,  1,     0,  2,  0,  2,   -301,    0,      129,   -0.1},
                {35,  1,     0,  0,  -2, 0,   -158,    0,      -1,    0},
                {36,  -1,    0,  2,  0,  2,   123,     0,      -53,   0},
                {37,  0,     0,  0,  2,  0,   63,      0,      -2,    0},
                {38,  1,     0,  0,  0,  1,   63,      0.1,    -33,   0},
                {39,  -1,    0,  0,  0,  1,   -58,     -0.1,   32,    0},
                {40,  -1,    0,  2,  2,  2,   -59,     0,      26,    0},
                {41,  1,     0,  2,  0,  1,   -51,     0,      27,    0},
                {42,  0,     0,  2,  2,  2,   -38,     0,      16,    0},
                {43,  2,     0,  0,  0,  0,   29,      0,      -1,    0},
                {44,  1,     0,  2,  -2, 2,   29,      0,      -12,   0},
                {45,  2,     0,  2,  0,  2,   -31,     0,      13,    0},
                {46,  0,     0,  2,  0,  0,   26,      0,      -1,    0},
                {47,  -1,    0,  2,  0,  1,   21,      0,      -10,   0},
                {48,  -1,    0,  0,  2,  1,   16,      0,      -8,    0},
                {49,  1,     0,  0,  -2, 1,   -13,     0,      7,     0},
                {50,  -1,    0,  2,  2,  1,   -10,     0,      5,     0},
                {51,  1,     1,  0,  -2, 0,   -7,      0,      0,     0},
                {52,  0,     1,  2,  0,  2,   7,       0,      -3,    0},
                {53,  0,     -1, 2,  0,  2,   -7,      0,      3,     0},
                {54,  1,     0,  2,  2,  2,   -8,      0,      3,     0},
                {55,  1,     0,  0,  2,  0,   6,       0,      0,     0},
                {56,  2,     0,  2,  -2, 2,   6,       0,      -3,    0},
                {57,  0,     0,  0,  2,  1,   -6,      0,      3,     0},
                {58,  0,     0,  2,  2,  1,   -7,      0,      3,     0},
                {59,  1,     0,  2,  -2, 1,   6,       0,      -3,    0},
                {60,  0,     0,  0,  -2, 1,   -5,      0,      3,     0},
                {61,  1,     -1, 0,  0,  0,   5,       0,      0,     0},
                {62,  2,     0,  2,  0,  1,   -5,      0,      3,     0},
                {63,  0,     1,  0,  -2, 0,   -4,      0,      0,     0},
                {64,  1,     0,  -2, 0,  0,   4,       0,      0,     0},
                {65,  0,     0,  0,  1,  0,   -4,      0,      0,     0},
                {66,  1,     1,  0,  0,  0,   -3,      0,      0,     0},
                {67,  1,     0,  2,  0,  0,   3,       0,      0,     0},
                {68,  1,     -1, 2,  0,  2,   -3,      0,      1,     0},
                {69,  -1,    -1, 2,  2,  2,   -3,      0,      1,     0},
                {70,  -2,    0,  0,  0,  1,   -2,      0,      1,     0},
                {71,  3,     0,  2,  0,  2,   -3,      0,      1,     0},
                {72,  0,     -1, 2,  2,  2,   -3,      0,      1,     0},
                {73,  1,     1,  2,  0,  2,   2,       0,      -1,    0},
                {74,  -1,    0,  2,  -2, 1,   -2,      0,      1,     0},
                {75,  2,     0,  0,  0,  1,   2,       0,      -1,    0},
                {76,  1,     0,  0,  0,  2,   -2,      0,      1,     0},
                {77,  3,     0,  0,  0,  0,   2,       0,      0,     0},
                {78,  0,     0,  2,  1,  2,   2,       0,      -1,    0},
                {79,  -1,    0,  0,  0,  2,   1,       0,      -1,    0},
                {80,  1,     0,  0,  -4, 0,   -1,      0,      0,     0},
                {81,  -2,    0,  2,  2,  2,   1,       0,      -1,    0},
                {82,  -1,    0,  2,  4,  2,   -2,      0,      1,     0},
                {83,  2,     0,  0,  -4, 0,   -1,      0,      0,     0},
                {84,  1,     1,  2,  -2, 2,   1,       0,      -1,    0},
                {85,  1,     0,  2,  2,  1,   -1,      0,      1,     0},
                {86,  -2,    0,  2,  4,  2,   -1,      0,      1,     0},
                {87,  -1,    0,  4,  0,  2,   1,       0,      0,     0},
                {88,  1,     -1, 0,  -2, 0,   1,       0,      0,     0},
                {89,  2,     0,  2,  -2, 1,   1,       0,      -1,    0},
                {90,  2,     0,  2,  2,  2,   -1,      0,      0,     0},
                {91,  1,     0,  0,  2,  1,   -1,      0,      0,     0},
                {92,  0,     0,  4,  -2, 2,   1,       0,      0,     0},
                {93,  3,     0,  2,  -2, 2,   1,       0,      0,     0},
                {94,  1,     0,  2,  -2, 0,   -1,      0,      0,     0},
                {95,  0,     1,  2,  0,  1,   1,       0,      0,     0},
                {96,  -1,    -1, 0,  2,  1,   1,       0,      0,     0},
                {97,  0,     0,  -2, 0,  1,   -1,      0,      0,     0},
                {98,  0,     0,  2,  -1, 2,   -1,      0,      0,     0},
                {99,  0,     1,  0,  2,  0,   -1,      0,      0,     0},
                {100, 1,     0,  -2, -2, 0,   -1,      0,      0,     0},
                {101, 0,     -1, 2,  0,  1,   -1,      0,      0,     0},
                {102, 1,     1,  0,  -2, 1,   -1,      0,      0,     0},
                {103, 1,     0,  -2, 2,  0,   -1,      0,      0,     0},
                {104, 2,     0,  0,  2,  0,   1,       0,      0,     0},
                {105, 0,     0,  2,  4,  2,   -1,      0,      0,     0},
                {106, 0,     1,  0,  1,  0,   1,       0,      0,     0}};

        double matlab[106][9];
        for (int i = 0; i < 9; i++) {
            for (int j = 0; j < 106; j++) {
                matlab[j][i] = aa[j][i + 1];
            }

        }
        double deg;
        deg = PI / 180;

        fai = 0;
        sigma = 0;


        double A[5], AA[5];
        AA[0] = 485866.733 + (1325 * 360 * 3600.0 + 715922.633) * T + 31.31 * T * T + 0.064 * T * T * T;
        AA[1] = 1287099.804 + (99 * 360 * 3600.0 + 1292581.224) * T - 0.577 * T * T - 0.012 * T * T * T;
        AA[2] = 335778.877 + (1342 * 360 * 3600.0 + 295263.137) * T - 13.257 * T * T + 0.011 * T * T * T;
        AA[3] = 1072261.307 + (1236 * 360 * 3600.0 + 1105601.328) * T - 6.891 * T * T + 0.019 * T * T * T;
        AA[4] = 450160.280 + (5 * 360 * 3600.0 + 482890.539) * T - 7.455 * T * T + 0.008 * T * T * T;

        for (int i = 0; i < 5; i++) {
            A[i] = AA[i] / 3600.0;
        }

        for (int j = 0; j < 106; j++) {
            double c, d, a;
            double n[5];

            c = (matlab[j][5] + matlab[j][6] * T) * (1e-4) / 3600;
            d = (matlab[j][7] + matlab[j][8] * T) * (1e-4) / 3600;

            for (int k = 0; k < 5; k++) {

                for (int ii = 0; ii < 5; ii++) {
                    n[ii] = matlab[j][ii];

                }
                a = tools.OnVecDotVec(n, A, 5);

            }
            fai = fai + c * sin(a * deg);
            sigma = sigma + d * cos(a * deg);


        }


        delta_sigma = sigma;
        delta_fai = fai;

    }
}
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
					
