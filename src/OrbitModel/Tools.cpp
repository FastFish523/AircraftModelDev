#include "OrbitModel/Tools.h"
#include <cstdlib>
#include <OrbitModel/Constant.h>

#include "OrbitModel/stdafx.h"

namespace JTC_Basic_OrbitModel {


    Tools::Tools() {
    }


    Tools::~Tools() {
    }


// basic rotation about x-axis
    void Tools::OnMatX(double **M, double x) {
        M[0][0] = 1;
        M[0][1] = 0;
        M[0][2] = 0;
        M[1][0] = 0;
        M[1][1] = cos(x);
        M[1][2] = sin(x);
        M[2][0] = 0;
        M[2][1] = -sin(x);
        M[2][2] = cos(x);
    }

// basic rotation about y-axis
    void Tools::OnMatY(double **M, double x) {
        M[0][0] = cos(x);
        M[0][1] = 0;
        M[0][2] = -sin(x);
        M[1][0] = 0;
        M[1][1] = 1;
        M[1][2] = 0;
        M[2][0] = sin(x);
        M[2][1] = 0;
        M[2][2] = cos(x);
    }

// basic rotation about z-axis
    void Tools::OnMatZ(double **M, double x) {
        M[0][0] = cos(x);
        M[0][1] = sin(x);
        M[0][2] = 0;
        M[1][0] = -sin(x);
        M[1][1] = cos(x);
        M[1][2] = 0;
        M[2][0] = 0;
        M[2][1] = 0;
        M[2][2] = 1;
    }


    void Tools::Unit(double *u_vec, double *vec) {
        double small = 0.000001;
        double magv;
        magv = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
        if (magv > small) {
            for (int i = 0; i < 3; i++)
                u_vec[i] = vec[i] / sqrt(magv);
        } else {
            for (int i = 0; i < 3; i++)
                u_vec[i] = 0.0;

        }

    }

    double Tools::Mag(double *vec) {
        double temp, mag;

        temp = vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];


        if (abs(temp) >= 1.0e-16)
            mag = sqrt(temp);
        else
            mag = 0.0;

        return mag;

    }


    double Tools::round(double r) {

        return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);

    }

    double Tools::OnNorm(double *x, int length) {
        double norm = 0;
        for (int i = 0; i < length; i++) {
            norm += x[i] * x[i];
        }
        return sqrt(norm);
    }


// M3 = M1 x M2 (3x3 Matrix)
    void Tools::OnMatByMat(double **M1, double **M2, double **M3) {
        double tmp[3][3];

        for (int i = 0; i < 3; i++) {
            for (int k = 0; k < 3; k++) {
                tmp[i][k] = 0;
                for (int j = 0; j < 3; j++) {
                    tmp[i][k] += M1[i][j] * M2[j][k];
                }
            }
        }
        for (int i = 0; i < 3; i++) {
            for (int k = 0; k < 3; k++) {
                M3[i][k] = tmp[i][k];
            }
        }
    }

// x[3] cross y[3] => n[3]
    void Tools::OnVecByVec(double *x, double *y, double *n) {
        double z[3];
        for (int i = 0; i < 3; i++) {
            z[i] = y[i];
        }
        n[0] = x[1] * z[2] - x[2] * z[1];
        n[1] = x[2] * z[0] - x[0] * z[2];
        n[2] = x[0] * z[1] - x[1] * z[0];
    }

    double Tools::OnVecDotVec(double *x, double *y, int len) {
        double ret = 0;

        for (int i = 0; i < len; i++) {
            ret += x[i] * y[i];
        }
        return ret;
    }

    void Tools::OnVecEqVec(double *x, double *y, int nD) {
        for (int i = 0; i < nD; i++) {
            y[i] = x[i];
        }
    }

    void Tools::OnVecSubVec(double *x, double *y, double *Ret, int nD) {
        for (int i = 0; i < nD; i++) {
            Ret[i] = x[i] - y[i];
        }
    }

    void Tools::OnVecAddVec(double *x, double *y, double *Ret, int nD) {
        for (int i = 0; i < nD; i++) {
            Ret[i] = x[i] + y[i];
        }
    }

// Y = M*X, M:3*3
    void Tools::OnMatDotVec(double **M, double *x, double *y) {
        double xw[3];
        for (int i = 0; i < 3; i++) {
            xw[i] = x[i];
        }

        for (int i = 0; i < 3; i++) {
            y[i] = 0;
            for (int j = 0; j < 3; j++)
                y[i] += M[i][j] * xw[j];
        }


    }

    void Tools::OnVecDotNum(double *x, double y, double *n) {
        for (int i = 0; i < 3; i++) {
            n[i] = y * x[i];
        }
    }

// Matrix transpose
    void Tools::OnMatTranspose(double **M, double **MT) {
        double tmp[3][3];

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                tmp[i][j] = M[i][j];

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                MT[i][j] = tmp[j][i];
    }

// definition of dynamic two dimensional array
    double **Tools::TwoArrayAlloc(int r, int c) {
        double *x, **y;
        int n;

        x = (double *) calloc(r * c, sizeof(double));
        y = (double **) calloc(r, sizeof(double *));
        for (n = 0; n <= r - 1; ++n)
            y[n] = &x[c * n];

        return (y);
    }

    void Tools::TwoArrayFree(double **x) {
        free(x[0]);
        free(x);
    }

    double Tools::R2P5(double y) {
        double x, d;
        x = floor(y);
        d = y - x;
        if (d >= 0.5) {
            x = x + 0.5;
        } else {
            x = x - 0.5;
        }

        return x;
    }

//矩阵求逆
    int Tools::MatrixInv(double *a, int n) {
        int *is, *js, i, j, k, l, u, v;
        double d, p;

        is = (int *) malloc(n * sizeof(int));
        js = (int *) malloc(n * sizeof(int));

        for (k = 0; k <= n - 1; k++) {
            d = 0.0;

            for (i = k; i <= n - 1; i++) {
                for (j = k; j <= n - 1; j++) {
                    l = i * n + j;
                    p = fabs(a[l]);
                    if (p > d) {
                        d = p;
                        is[k] = i;
                        js[k] = j;
                    }
                }
            }

            if (d + 1.0 == 1.0) {
                free(is);
                free(js);
                //printf("err**not inv\n");
                return (0);
            }

            if (is[k] != k) {
                for (j = 0; j <= n - 1; j++) {
                    u = k * n + j;
                    v = is[k] * n + j;
                    p = a[u];
                    a[u] = a[v];
                    a[v] = p;
                }
            }

            if (js[k] != k) {
                for (i = 0; i <= n - 1; i++) {
                    u = i * n + k;
                    v = i * n + js[k];
                    p = a[u];
                    a[u] = a[v];
                    a[v] = p;
                }
            }

            l = k * n + k;
            a[l] = 1.0 / a[l];

            for (j = 0; j <= n - 1; j++) {
                if (j != k) {
                    u = k * n + j;
                    a[u] = a[u] * a[l];
                }
            }

            for (i = 0; i <= n - 1; i++) {
                if (i != k)
                    for (j = 0; j <= n - 1; j++)
                        if (j != k) {
                            u = i * n + j;
                            a[u] = a[u] - a[i * n + k] * a[k * n + j];
                        }
            }

            for (i = 0; i <= n - 1; i++) {
                if (i != k) {
                    u = i * n + k;
                    a[u] = -a[u] * a[l];
                }
            }
        }

        for (k = n - 1; k >= 0; k--) {
            if (js[k] != k) {
                for (j = 0; j <= n - 1; j++) {
                    u = k * n + j;
                    v = js[k] * n + j;
                    p = a[u];
                    a[u] = a[v];
                    a[v] = p;
                }
            }

            if (is[k] != k) {
                for (i = 0; i <= n - 1; i++) {
                    u = i * n + k;
                    v = i * n + is[k];
                    p = a[u];
                    a[u] = a[v];
                    a[v] = p;
                }
            }
        }
        free(is);
        free(js);

        // 	for (i=0;i<3;i++)	//use
        // 	{
        // 		for (j=0;j<3;j++)
        // 		{
        // 			b[i][j]=a[i][j];
        // 		}
        // 	}

        return (1);
    }


    CartesianCoordinates Tools::BLHtoXYZ(GeodeticCoordinates geodetic) {

        double dblD2R = M_PI / 180;
        double e1 = sqrt(pow(aAxis, 2) - pow(bAxis, 2)) / aAxis;
        double B, L, H;
        H = geodetic.alt;
        B = geodetic.lat;
        L = geodetic.lon;

        double N = aAxis / sqrt(1.0 - pow(e1, 2) * pow(sin(B * dblD2R), 2));
        CartesianCoordinates cartesian;
        cartesian.x = (N + H) * cos(B * dblD2R) * cos(L * dblD2R);
        cartesian.y = (N + H) * cos(B * dblD2R) * sin(L * dblD2R);
        cartesian.z = (N * (1.0 - pow(e1, 2)) + H) * sin(B * dblD2R);
        return cartesian;
    }


    GeodeticCoordinates Tools::XYZtoBLH(CartesianCoordinates cartesian) {
        GeodeticCoordinates geodetic;
        double e1 = (pow(aAxis, 2) - pow(bAxis, 2)) / pow(aAxis, 2);
        double e2 = (pow(aAxis, 2) - pow(bAxis, 2)) / pow(bAxis, 2);

        double X, Y, Z;
        X = cartesian.x;
        Y = cartesian.y;
        Z = cartesian.z;

        double S = sqrt(pow(X, 2) + pow(Y, 2));
        double cosL = X / S;
        double B = 0;
        double L = 0;

        //L = acos(cosL);
        //L = fabs(L);
		L=atan2(Y,X);

        double tanB = Z / S;
        //B = atan(tanB);
		B=atan2(Z,S);
        double c = aAxis * aAxis / bAxis;
        double preB0 = 0.0;
        double ll = 0.0;
        double N = 0.0;
        //迭代计算纬度
        do {
            preB0 = B;
            ll = pow(cos(B), 2) * e2;
            N = c / sqrt(1 + ll);

            tanB = (Z + N * e1 * sin(B)) / S;
            // B = atan(tanB);
			B= atan2((Z + N * e1 * sin(B)),S);
        } while (fabs(preB0 - B) >= 0.0000000001);

        ll = pow(cos(B), 2) * e2;
        N = c / sqrt(1 + ll);
        if (B == 0) {
            geodetic.alt = S / cos(B) - N;
        } else {
            geodetic.alt = Z / sin(B) - N * (1 - e1);
        }

        geodetic.lat = B * 180 / M_PI;
        geodetic.lon = L * 180 / M_PI;

        return geodetic;
    }

}
