#pragma once

#include <cmath>
namespace JTC_Basic_OrbitModel {
    using namespace std;
#define aAxis 6378137.0
#define bAxis 6356755.28856

    typedef struct {
        double x;
        double y;
        double z;
    } CartesianCoordinates;

    typedef struct {
        double lon;    // 经度
        double lat;    // 纬度
        double alt;    // 海拔高度
    } GeodeticCoordinates;

    class Tools {
    public:
        Tools();

        ~Tools();

        ////基本运算
        double OnNorm(double *x, int length);

        void OnVecDotNum(double *x, double y, double *n);

        double OnVecDotVec(double *x, double *y, int len);

        void OnVecByVec(double *x, double *y, double *n);

        void OnVecAddVec(double *x, double *y, double *Ret, int nD);

        void OnVecSubVec(double *x, double *y, double *Ret, int nD);

        void OnVecEqVec(double *x, double *y, int nD);

        void OnMatByMat(double **M1, double **M2, double **M3);

        void OnMatDotVec(double **M, double *x, double *y);

        void OnMatTranspose(double **M, double **MT);

        double **TwoArrayAlloc(int r, int c);

        void TwoArrayFree(double **x);

        void Unit(double *u_vec, double *vec);

        double Mag(double *vec);

        double round(double r);

        double R2P5(double y);
        ////坐标变换

        int MatrixInv(double *a, int n);

        void OnMatX(double **M, double x);

        void OnMatY(double **M, double x);

        void OnMatZ(double **M, double x);

        CartesianCoordinates BLHtoXYZ(GeodeticCoordinates geodetic);

        GeodeticCoordinates XYZtoBLH(CartesianCoordinates cartesian);

    private:

    };

}
