#include "OrbitModel/Kepler.h"
#include "math.h"
#include <OrbitModel/Constant.h>

#include "OrbitModel/stdafx.h"
namespace JTC_Basic_OrbitModel {
    Kepler::Kepler() {

    }

    Kepler::~Kepler() {

    }

//! 解开普勒方程 M = E - e*sin(E)
// 即已知e和M，找到E满足开普勒方程
// 采用牛顿迭代法解方程，定义函数:
// f(E) = E - e*sin(E) - M
// 则 f'(E) = 1 - e*cos(E)
// 通过下式进行迭代计算E:
//                  f(E|k)
// E|k+1 = E|k - ----------
//                 f'(E|k)
// 对于双曲线轨道，开普勒方程为 M = e*sin(E) - E，解法类似

    double Kepler::KeplerFunc(double ee, double MM) {
        double E0 = MM;
        double E1;
        if (ee <= 1.0) E1 = E0 - (E0 - ee * sin(E0) - MM) / (1 - ee * cos(E0));
        else E1 = E0 - (ee * sinh(E0) - E0 - MM) / (ee * cosh(E0) - 1.0);
        while (fabs(E1 - E0) > 1e-8) {
            E0 = E1;
            if (ee <= 1.0) E1 = E0 - (E0 - ee * sin(E0) - MM) / (1 - ee * cos(E0));
            else E1 = E0 - (ee * sinh(E0) - E0 - MM) / (ee * cosh(E0) - 1.0);
        }
        return E1;
    }

    double Kepler::Kepler_U(double dt, double ro, double vro, double a) {

        double error, x, ratio, C, S, F, dFdx;
        int nMax, n;

        error = 1.e-8;
        nMax = 1000;

        x = sqrt(mu) * abs(a) * dt;
        n = 0;
        ratio = 1;
        while (abs(ratio) > error && n <= nMax) {
            n = n + 1;
            C = stumpC(a * pow(x, 2));
            S = stumpS(a * pow(x, 2));
            F = ro * vro / sqrt(mu) * pow(x, 2) * C + (1 - a * ro) * pow(x, 3) * S + ro * x - sqrt(mu) * dt;
            dFdx = ro * vro / sqrt(mu) * x * (1 - a * pow(x, 2) * S) + (1 - a * ro) * pow(x, 2) * C + ro;
            ratio = F / dFdx;
            x = x - ratio;
        }
        return x;
    }


    double Kepler::f(double x, double t, double ro, double a) {
        double z, f;
        z = a * pow(x, 2);
        f = 1 - pow(x, 2) / ro * stumpC(z);
        return f;
    }

    double Kepler::g(double x, double t, double ro, double a) {
        double z, g;
        z = a * pow(x, 2);
        g = t - 1 / sqrt(mu) * pow(x, 3) * stumpS(z);
        return g;
    }

    double Kepler::fdot(double x, double r, double ro, double a) {
        double z, fdot;
        z = a * pow(x, 2);
        fdot = sqrt(mu) / r / ro * (z * stumpS(z) - 1) * x;

        return fdot;
    }

    double Kepler::gdot(double x, double r, double ro, double a) {
        double z, gdot;
        z = a * pow(x, 2);
        gdot = 1 - pow(x, 2) / r * stumpC(z);

        return gdot;
    }

    double Kepler::y(double r1, double r2, double A, double z) {
        double dum;
        dum = r1 + r2 + A * (z * S(z) - 1) / sqrt(C(z));

        return dum;

    }

    double Kepler::F(double r1, double r2, double A, double z, double t) {
        double dum;
        if (y(r1, r2, A, z) < 0) {
            dum = -1;
        } else {
            dum = pow((y(r1, r2, A, z) / C(z)), 1.5) * S(z) + A * sqrt(y(r1, r2, A, z)) - sqrt(mu) * t;
        }


        return dum;

    }

    double Kepler::dFdz(double r1, double r2, double A, double z, double t) {
        double dum;
        if (z == 0) {
            dum = sqrt(2.0) / 40 * pow(y(r1, r2, A, 0), 1.5) +
                  A / 8 * (sqrt(y(r1, r2, A, 0)) + A * sqrt(1 / 2 / y(r1, r2, A, 0)));

        } else {
            dum = pow(y(r1, r2, A, z) / C(z), 1.5) *
                  (1 / 2 / z * (C(z) - 3 * S(z) / 2 / C(z)) + 3 * pow(S(z), 2) / 4 / C(z)) +
                  A / 8 * (3 * S(z) / C(z) * sqrt(y(r1, r2, A, z)) + A * sqrt(C(z) / y(r1, r2, A, z)));
        }

        return dum;

    }


    double Kepler::C(double z) {
        return stumpC(z);
    }

    double Kepler::S(double z) {
        return stumpS(z);
    }

    double Kepler::stumpS(double z) {
        double s;
        if (z > 0) {
            s = (sqrt(z) - sin(sqrt(z))) / pow(sqrt(z), 3);
        } else if ((z < 0)) {
            s = (sinh(sqrt(-z)) - sqrt(-z)) / pow(sqrt(-z), 3);
        } else {
            s = 1 / 6;
        }
        return s;

    }

    double Kepler::stumpC(double z) {
        double c;
        if (z > 0) {
            c = (1 - cos(sqrt(z))) / z;
        } else if ((z < 0)) {
            c = (cosh(sqrt(-z)) - 1) / (-z);
        } else {
            c = 1 / 2;
        }
        return c;
    }

}
