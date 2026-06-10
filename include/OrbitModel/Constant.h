#if !defined(AFX_DEFINES_H_INCLUDED_)
#define AFX_DEFINES_H_INCLUDED_

namespace JTC_Basic_OrbitModel {
// math functions                                                             
#define MIN(A, B)        ((A) < (B) ? (A) : (B))
#define MAX(A, B)        ((A) > (B) ? (A) : (B))
#define SQR(x)            ((x)*(x))
#define fix_angle(a)        ((a) - 360.0 * (floor((a) / 360.0)))
#define to_rad(d)        ((d) * (PI / 180.0))
#define to_deg(d)        ((d) * (180.0 / PI))
#define RADIAN(d)        ((d) * (PI / 180.0))
#define DEGREE(d)        ((d) * (180.0 / PI))


constexpr double RAD = 0.0174532925199433; //degree to radian
constexpr double DEG = 57.2957795130823 ;  //radian to degree
/******************************************************************************/
/*                                                                            */
/* general numerical constants                                                */
/*                                                                            */
/******************************************************************************/

    constexpr double FOURPI =       (4.0*3.141592653);
    constexpr double TWOPI  =       (2.0*3.141592653);
#ifndef HALFPI
constexpr double HALFPI         =       (0.5*3.141592653);
#endif

constexpr double THREEHALFPI     =      (1.5*3.141592653);

constexpr double ZERO       =     0.0;
constexpr double ONEPPB     =     1.0e-9;
constexpr double ONEPPM     =     1.0e-6;
constexpr double TWOPPM     =     2.0e-6;
constexpr double ONETHIRD   =     (1.0/3.0);
constexpr double TWOTHIRDS  =     (2.0/3.0);
constexpr double THREEHALFS =     (3.0/2.0);
constexpr double ONE        =     1.0;
constexpr double ONEMEG     =     1.0e6;
constexpr double TWOMEG     =     2.0e6;

/******************************************************************************/
/*                                                                            */
/* numerical constants for unit conversions                                   */
/*                                                                            */
/******************************************************************************/
    constexpr double CRH   =       (24.0/TWOPI)     ;    /* convert rad into hours  */
    constexpr double CRS   =       (86400.0/TWOPI)  ;    /* convert rad into sec    */
    constexpr double CRD   =       (360.0/TWOPI)    ;    /* convert rad into deg    */
    constexpr double CRAM  =       (21600.0/TWOPI)   ;   /* convert rad into arcmin */
    constexpr double CRAS  =       (1296000.0/TWOPI)  ;  /* convert rad into arcsec */
    constexpr double CDR   =       (TWOPI/360.0)      ;  /* convert deg into rad    */
    constexpr double CAMR  =       (TWOPI/21600.0)    ;  /* convert arcmin into rad */
    constexpr double CASR  =       (TWOPI/1296000.0)   ; /* convert arcsec into rad */
    constexpr double CRREV =       (1.0/TWOPI)        ;  /* convert rad into rev    */

    constexpr double HALFDEG        =       (0.5*CDR)        ;    /* half a deg [rad]        */

    constexpr double MPD  =       1440.0           ; /* minutes per day         */
    constexpr double MPD2 =       (MPD*MPD)     ;    /* (minutes per day)^2     */
    constexpr double MPD3 =       (MPD2*MPD)     ;   /* (minutes per day)^3     */
    constexpr double SPD  =       86400.0        ;   /* seconds per day         */

    constexpr double CKMM          =        1000.0         ;     /* convert km to m         */
    constexpr double CMKM          =        1.0e-3         ;     /* convert m to km         */
    constexpr double CKMNM         =        0.539956804    ;     /* convert km to naut. mil.*/
    constexpr double CHZKHZ        =        1.0e-3         ;     /* convert Hz to kHz       */
    constexpr double CKHZMHZ       =        1.0e-3         ;     /* convert kHz to MHz      */
    constexpr double CHZMHZ        =        1.0e-6         ;     /* convert Hz to MHz       */
    constexpr double CKHZHZ        =        1.0e+3         ;     /* convert kHz to Hz       */
    constexpr double CMHZHZ        =        1.0e+6         ;     /* convert MHz to Hz       */

/******************************************************************************/
/*                                                                            */
/* numerical constants describing the Earth's orbit and figure                */
/*                                                                            */
/* EARTHRADIUS and EARTHFLAT are from: "Astronomical Almanac", 1991, p. K13   */
/*                                                                            */
/******************************************************************************/

//注：长度单位均为:km
#ifndef RS
constexpr double RS =696000;
#endif

#ifndef RE
constexpr double  RE =6378.137;
#endif

#ifndef RM
constexpr double  RM =1738.2;
#endif

#ifndef DIS_SE
constexpr double   DIS_SE =1.496e+8;
#endif

#ifndef DIS_ME
constexpr double   DIS_ME =3.844e+5;
#endif

//#ifndef GM
//	#define  GM 398600.5
//#endif

#ifndef PI
constexpr double  PI = 3.1415926535897932384626433832795;
#endif

#ifndef M_PI
constexpr double M_PI =  3.1415926535897932384626433832795;
#endif

#ifndef J2
constexpr double J2 = 0.00108263;
#endif

#ifndef J3
constexpr double J3 = -0.00000254;
#endif

#ifndef J4
constexpr double J4 = -0.00000161;
#endif

/*  Properties of the Earth  */
    constexpr double EARTHSMA         =     149597892.0    ;      /* 1 AU [km]               */
    constexpr double EARTHRADIUS      =     6378.137      ;       /* equatorial radius [km]  */
    constexpr double EARTHECCEN       =     0.01675104     ;      /* Earth's orbit eccentr.  */
    constexpr double EARTHFLAT        =     (1.0/298.257222)  ;   /* geoid model parameters  */
    constexpr double earthrad         =     6378.137     ;   /* Radius of Earth in kilometres */
    constexpr double EarthFlat        =    (1/298.257222) ;           /* Earth Flattening Coeff. */
    constexpr double mu          =          398600.4418 ;    //WGS84  geocentric gravitational constant (km^3/s^2)
//#define GM                    398600.5		       /* [km^3/s^2]              */
    constexpr double GM  =                  398600.4418;               /* [km^3/s^2]              */

#ifndef OMEGAE
    constexpr double OMEGAE      =          7.292116E-05;
#endif

#define g_earth               9.80665   // 地球表面的引力加速度常数(m/s^2)
    constexpr double EarthMass          =   5.974242e24 ; // earth mass(kg)
#define We                    7.2921159e-5   //earth ratation rate(Rad/s)
/******************************************************************************/
/*                                                                            */
/* numerical constants describing the apparent size of the Sun and the        */
/* proximity limit for calculating transits across the solar disk             */
/*                                                                            */
/******************************************************************************/
    constexpr double SUNRADIUS      =      695980.0         ;    /* equatorial radius [km]  */
    constexpr double SUNDISKRAD     =      (16.0*CAMR)      ;    /* Sun disk radius [rad]   */
    constexpr double SUNPROX        =      48.0            ;     /* Sun prox limit [arcmin] */
    constexpr double GS             =      1.32712440018e+11 ;//heliocentric gravitational constant (km^3/s^2)
    constexpr double SunMass        =      1.9889e30 ; //太阳质量(kg)
//#define Rs                    695990.0        //太阳半径(光球层?)(km)
    constexpr double M_sunD         =           1.99096875e-7;//太阳平近点角变化率(rad/s)

/******************************************************************************/
/*                                                                            */
/* numerical constants describing the apparent size of the Moon and the        */
/* proximity limit for calculating transits across the solar disk             */
/*                                                                            */
/******************************************************************************/
//#define  GM                 4902.801056     //selenocentric gravitational constant (km^3/s^2)
    constexpr double  Rm              =   1738.0  ;  // 月球平均赤道半径
    constexpr double  LunarGrvSph     =   66200   ; //月球影响球半径(km)
/******************************************************************************/
/*                                                                            */
/* numerical constants describing the motions within the solar system         */
/*                                                                            */
/******************************************************************************/
    constexpr double JULCENT          =     36525.0           ;   /* mean solar days / jcy   */
    constexpr double TROPCENT         =     36524.219879       ;  /* mean solar days / cy    */
    constexpr double TROPYEAR         =     (TROPCENT/100.0)   ;  /* mean solar days / year  */
    constexpr double JULDAT1900       =     2415020.0          ;  /* Julian date of 1900.0   */
    constexpr double JULDAT1950       =     2433282.423        ;  /* Julian date of 1950.0   */
    constexpr double JULDAT2000       =     2451545.0          ;  /* Julian date of 2000.0   */
    constexpr double SIDSOLAR         =     1.002737909350      ; /* sidereal rotation rate  */
    constexpr double SIDRATE          =     (TWOPI/SPD*SIDSOLAR) ;/* [rad/s]                 */

/******************************************************************************/
/*                                                                            */
/* physical constants                                                         */
/*                                                                            */
/* GM (and KEPLER) are from: "Astronomical Almanac", 1991, p. K13             */
/*                                                                            */
/******************************************************************************/
    constexpr double CVAC            =     2.99792458e5    ;     /* speed of light [km/s]   */
    constexpr double GMSGP           =     398600.7995      ;    /* value used in SGP model */
    constexpr double KEPLER          =     42241.09773      ;    /* GM^(1/3)*(SPD/2PI)^(2/3)*/
    constexpr double KEPLERSGP       =     42241.10831       ;   /* value used in SGP model */
    /* with a [km] and T [min] */
/* Julian date at standard epoch */
    constexpr double  J2000          =  2451545.0;
    constexpr double  TO_CENTURIES   =   36525.0;
    constexpr double  SEC_IN_DAY     =  86400.0;
    constexpr double  HRS_IN_RADIAN  =  3.819718634205;
    constexpr double  DEG_IN_RADIAN  =  57.2957795130823;
/* flattening of earth, 1/298.257 */
    constexpr double  FLATTEN    =       0.003352813;
/* equatorial radius of earth, meters */
    constexpr double  EQUAT_RAD   =      6378137.0;
/* 1 AU in meters */
    constexpr double  ASTRO_UNIT    =    1.4959787066e11;

/*  Astronomical constants  */

#define epoch       2444238.5      /* 1980 January 0.0 */

/*  Constants defining the Sun's apparent orbit  */
    constexpr double elonge    = 278.833540  ;   /* Ecliptic longitude of the Sun at epoch 1980.0 */
    constexpr double elongp    = 282.596403  ;   /* Ecliptic longitude of the Sun at perigee */
    constexpr double eccent    = 0.016718    ;   /* Eccentricity of Earth's orbit */
    constexpr double sunsmax   = 1.495985e8   ;  /* Semi-major axis of Earth's orbit, km */
    constexpr double sunangsiz = 0.533128      ; /* Sun's angular size, degrees, at
                                      semi-major axis distance */

/*  Elements of the Moon's orbit, epoch 1980.0  */
    constexpr double mmlong    = 64.975464   ;   /* Moon's mean longitude at the epoch */
    constexpr double mmlongp   = 349.383063  ;   /* Mean longitude of the perigee at the  epoch */
    constexpr double mlnode    = 151.950429  ;   /* Mean longitude of the node at the  epoch */
    constexpr double minc      = 5.145396    ;   /* Inclination of the Moon's orbit */
    constexpr double mecc      = 0.054900    ;   /* Eccentricity of the Moon's orbit */
    constexpr double mangsiz   = 0.5181      ;   /* Moon's angular size at distance a  from Earth */
    constexpr double msmax     = 384401.0    ;   /* Semi-major axis of Moon's orbit in km */
    constexpr double mparallax = 0.9507      ;   /* Parallax at distance a from Earth */
    constexpr double synmonth  = 29.53058868 ;   /* Synodic month (new Moon to new Moon) */
    constexpr double lunatbase = 2423436.0   ;   /* A_BaseCommon date for E. W. Brown's numbered series of lunations (1923 January 16) */


#define WM_USER_CREATEORBIT WM_USER+101
#define WM_USER_DELETEORBIT WM_USER+102
#define WM_USER_CREATEORBITSS WM_USER+103
#define WM_USER_DELETEORBITSS WM_USER+104
#define WM_USER_SHOWCONTROLBAR WM_USER+105
#define WM_USER_RELOADMODEL WM_USER+106

#define NEWCREATEDSAT 0
#define SGP4SDP4SAT   1


}


#endif
