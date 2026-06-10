#pragma once
#include "stdafx.h"
//using namespace Constant;
namespace JTC_Basic_OrbitModel {
    class Kepler {
    public:

        double KeplerFunc(double ee, double MM);

        double Kepler_U(double dt, double ro, double vro, double a);

        double f(double x, double t, double ro, double a);

        double g(double x, double t, double ro, double a);

        double fdot(double x, double r, double ro, double a);

        double gdot(double x, double r, double ro, double a);

        double y(double r1, double r2, double A, double z);

        double F(double r1, double r2, double A, double z, double t);

        double dFdz(double r1, double r2, double A, double z, double t);

        double C(double z);

        double S(double z);


        double stumpS(double z);

        double stumpC(double z);

        Kepler(void);

        ~Kepler(void);

    };

}
