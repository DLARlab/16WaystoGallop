/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:56 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FL_thigh_joint.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }


/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t2392;
  double t4575;
  double t8245;
  double t6654;
  double t8264;
  double t2622;
  double t8243;
  double t8266;
  double t8270;
  double t8273;
  double t8287;
  double t8288;
  double t8291;
  double t313;
  double t2493;
  double t8299;
  double t8319;
  double t8320;
  double t8321;
  double t914;
  double t918;
  double t1338;
  double t2049;
  double t3037;
  double t4384;
  double t8274;
  double t8279;
  double t8280;
  double t8283;
  double t8353;
  double t8355;
  double t8357;
  double t8360;
  double t8364;
  double t8367;
  double t8300;
  double t8317;
  double t8378;
  double t8389;
  double t8393;
  double t8416;
  double t8419;
  double t8420;
  t2392 = Cos(var1[3]);
  t4575 = Cos(var1[5]);
  t8245 = Sin(var1[3]);
  t6654 = Sin(var1[4]);
  t8264 = Sin(var1[5]);
  t2622 = Sin(var1[9]);
  t8243 = t2392*t4575*t6654;
  t8266 = t8245*t8264;
  t8270 = t8243 + t8266;
  t8273 = Cos(var1[9]);
  t8287 = -1.*t4575*t8245;
  t8288 = t2392*t6654*t8264;
  t8291 = t8287 + t8288;
  t313 = Cos(var1[10]);
  t2493 = Cos(var1[4]);
  t8299 = Sin(var1[10]);
  t8319 = t8273*t8270;
  t8320 = -1.*t2622*t8291;
  t8321 = t8319 + t8320;
  t914 = -1.*t313;
  t918 = 1. + t914;
  t1338 = 0.183*t918;
  t2049 = 0. + t1338;
  t3037 = -0.047*t2622;
  t4384 = 0. + t3037;
  t8274 = -1.*t8273;
  t8279 = 1. + t8274;
  t8280 = 0.047*t8279;
  t8283 = 0. + t8280;
  t8353 = t4575*t8245*t6654;
  t8355 = -1.*t2392*t8264;
  t8357 = t8353 + t8355;
  t8360 = t2392*t4575;
  t8364 = t8245*t6654*t8264;
  t8367 = t8360 + t8364;
  t8300 = 0.183*t8299;
  t8317 = 0. + t8300;
  t8378 = t8273*t8357;
  t8389 = -1.*t2622*t8367;
  t8393 = t8378 + t8389;
  t8416 = t8273*t2493*t4575;
  t8419 = -1.*t2493*t2622*t8264;
  t8420 = t8416 + t8419;

  p_output1(0)=0. + t2049*t2392*t2493 + t4384*t8270 + t8283*t8291 + 0.13205*(t2622*t8270 + t8273*t8291) + t8317*t8321 + 0.183*(t2392*t2493*t313 - 1.*t8299*t8321) + var1(0);
  p_output1(1)=0. + t2049*t2493*t8245 + t4384*t8357 + t8283*t8367 + 0.13205*(t2622*t8357 + t8273*t8367) + t8317*t8393 + 0.183*(t2493*t313*t8245 - 1.*t8299*t8393) + var1(1);
  p_output1(2)=0. + t2493*t4384*t4575 - 1.*t2049*t6654 + 0.13205*(t2493*t2622*t4575 + t2493*t8264*t8273) + t2493*t8264*t8283 + t8317*t8420 + 0.183*(-1.*t313*t6654 - 1.*t8299*t8420) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_FL_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_FL_thigh_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



