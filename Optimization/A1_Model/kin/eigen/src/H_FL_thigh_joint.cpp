/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:58 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FL_thigh_joint.h"

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
static void output1(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t438;
  double t2187;
  double t3260;
  double t3037;
  double t3528;
  double t414;
  double t449;
  double t1262;
  double t1900;
  double t4169;
  double t3051;
  double t3649;
  double t4144;
  double t4194;
  double t4195;
  double t4250;
  double t8270;
  double t8280;
  double t8325;
  double t8334;
  double t8359;
  double t8360;
  double t4153;
  double t4463;
  double t4555;
  double t8331;
  double t8364;
  double t8369;
  double t8378;
  double t8402;
  double t8408;
  double t8412;
  double t8413;
  double t8416;
  double t495;
  double t4601;
  double t4843;
  double t8462;
  double t8463;
  double t8464;
  double t8468;
  double t8470;
  double t8471;
  double t8475;
  double t8476;
  double t8477;
  double t8478;
  double t8424;
  double t8429;
  double t8431;
  double t8482;
  double t8485;
  double t6301;
  double t8371;
  double t8374;
  double t8432;
  double t8435;
  double t8437;
  double t8375;
  double t8409;
  double t8411;
  t438 = Cos(var1[3]);
  t2187 = Cos(var1[5]);
  t3260 = Sin(var1[3]);
  t3037 = Sin(var1[4]);
  t3528 = Sin(var1[5]);
  t414 = Cos(var1[10]);
  t449 = Cos(var1[4]);
  t1262 = Sin(var1[10]);
  t1900 = Cos(var1[9]);
  t4169 = Sin(var1[9]);
  t3051 = t438*t2187*t3037;
  t3649 = t3260*t3528;
  t4144 = t3051 + t3649;
  t4194 = -1.*t2187*t3260;
  t4195 = t438*t3037*t3528;
  t4250 = t4194 + t4195;
  t8270 = t2187*t3260*t3037;
  t8280 = -1.*t438*t3528;
  t8325 = t8270 + t8280;
  t8334 = t438*t2187;
  t8359 = t3260*t3037*t3528;
  t8360 = t8334 + t8359;
  t4153 = t1900*t4144;
  t4463 = -1.*t4169*t4250;
  t4555 = t4153 + t4463;
  t8331 = t1900*t8325;
  t8364 = -1.*t4169*t8360;
  t8369 = t8331 + t8364;
  t8378 = t1900*t449*t2187;
  t8402 = -1.*t449*t4169*t3528;
  t8408 = t8378 + t8402;
  t8412 = t4169*t4144;
  t8413 = t1900*t4250;
  t8416 = t8412 + t8413;
  t495 = t414*t438*t449;
  t4601 = -1.*t1262*t4555;
  t4843 = t495 + t4601;
  t8462 = -1.*t414;
  t8463 = 1. + t8462;
  t8464 = 0.183*t8463;
  t8468 = 0. + t8464;
  t8470 = -0.047*t4169;
  t8471 = 0. + t8470;
  t8475 = -1.*t1900;
  t8476 = 1. + t8475;
  t8477 = 0.047*t8476;
  t8478 = 0. + t8477;
  t8424 = t4169*t8325;
  t8429 = t1900*t8360;
  t8431 = t8424 + t8429;
  t8482 = 0.183*t1262;
  t8485 = 0. + t8482;
  t6301 = t414*t449*t3260;
  t8371 = -1.*t1262*t8369;
  t8374 = t6301 + t8371;
  t8432 = t449*t2187*t4169;
  t8435 = t1900*t449*t3528;
  t8437 = t8432 + t8435;
  t8375 = -1.*t414*t3037;
  t8409 = -1.*t1262*t8408;
  t8411 = t8375 + t8409;

  p_output1(0)=t4843;
  p_output1(1)=t8374;
  p_output1(2)=t8411;
  p_output1(3)=0.;
  p_output1(4)=t8416;
  p_output1(5)=t8431;
  p_output1(6)=t8437;
  p_output1(7)=0.;
  p_output1(8)=t1262*t438*t449 + t414*t4555;
  p_output1(9)=t1262*t3260*t449 + t414*t8369;
  p_output1(10)=-1.*t1262*t3037 + t414*t8408;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.183*t4843 + 0.13205*t8416 + t438*t449*t8468 + t4144*t8471 + t4250*t8478 + t4555*t8485 + var1(0);
  p_output1(13)=0. + 0.183*t8374 + 0.13205*t8431 + t3260*t449*t8468 + t8325*t8471 + t8360*t8478 + t8369*t8485 + var1(1);
  p_output1(14)=0. + 0.183*t8411 + 0.13205*t8437 - 1.*t3037*t8468 + t2187*t449*t8471 + t3528*t449*t8478 + t8408*t8485 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_FL_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_FL_thigh_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



