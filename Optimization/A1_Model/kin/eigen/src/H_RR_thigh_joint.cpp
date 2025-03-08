/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:10 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RR_thigh_joint.h"

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
  double t193;
  double t1292;
  double t1561;
  double t1462;
  double t1583;
  double t66;
  double t369;
  double t419;
  double t838;
  double t2028;
  double t1505;
  double t1593;
  double t1655;
  double t2044;
  double t2094;
  double t2459;
  double t3110;
  double t3123;
  double t3342;
  double t3352;
  double t3355;
  double t3356;
  double t1839;
  double t2482;
  double t2696;
  double t3344;
  double t3357;
  double t3358;
  double t3425;
  double t3593;
  double t3664;
  double t3874;
  double t3922;
  double t4088;
  double t406;
  double t2703;
  double t2989;
  double t8711;
  double t8721;
  double t8748;
  double t8750;
  double t8754;
  double t8756;
  double t8783;
  double t8788;
  double t8789;
  double t8798;
  double t4163;
  double t4472;
  double t5097;
  double t8811;
  double t8812;
  double t3098;
  double t3362;
  double t3422;
  double t5427;
  double t5782;
  double t6200;
  double t3424;
  double t3698;
  double t3812;
  t193 = Cos(var1[3]);
  t1292 = Cos(var1[5]);
  t1561 = Sin(var1[3]);
  t1462 = Sin(var1[4]);
  t1583 = Sin(var1[5]);
  t66 = Cos(var1[13]);
  t369 = Cos(var1[4]);
  t419 = Sin(var1[13]);
  t838 = Cos(var1[12]);
  t2028 = Sin(var1[12]);
  t1505 = t193*t1292*t1462;
  t1593 = t1561*t1583;
  t1655 = t1505 + t1593;
  t2044 = -1.*t1292*t1561;
  t2094 = t193*t1462*t1583;
  t2459 = t2044 + t2094;
  t3110 = t1292*t1561*t1462;
  t3123 = -1.*t193*t1583;
  t3342 = t3110 + t3123;
  t3352 = t193*t1292;
  t3355 = t1561*t1462*t1583;
  t3356 = t3352 + t3355;
  t1839 = t838*t1655;
  t2482 = -1.*t2028*t2459;
  t2696 = t1839 + t2482;
  t3344 = t838*t3342;
  t3357 = -1.*t2028*t3356;
  t3358 = t3344 + t3357;
  t3425 = t838*t369*t1292;
  t3593 = -1.*t369*t2028*t1583;
  t3664 = t3425 + t3593;
  t3874 = t2028*t1655;
  t3922 = t838*t2459;
  t4088 = t3874 + t3922;
  t406 = t66*t193*t369;
  t2703 = -1.*t419*t2696;
  t2989 = t406 + t2703;
  t8711 = -1.*t66;
  t8721 = 1. + t8711;
  t8748 = -0.183*t8721;
  t8750 = 0. + t8748;
  t8754 = 0.047*t2028;
  t8756 = 0. + t8754;
  t8783 = -1.*t838;
  t8788 = 1. + t8783;
  t8789 = -0.047*t8788;
  t8798 = 0. + t8789;
  t4163 = t2028*t3342;
  t4472 = t838*t3356;
  t5097 = t4163 + t4472;
  t8811 = -0.183*t419;
  t8812 = 0. + t8811;
  t3098 = t66*t369*t1561;
  t3362 = -1.*t419*t3358;
  t3422 = t3098 + t3362;
  t5427 = t369*t1292*t2028;
  t5782 = t838*t369*t1583;
  t6200 = t5427 + t5782;
  t3424 = -1.*t66*t1462;
  t3698 = -1.*t419*t3664;
  t3812 = t3424 + t3698;

  p_output1(0)=t2989;
  p_output1(1)=t3422;
  p_output1(2)=t3812;
  p_output1(3)=0.;
  p_output1(4)=t4088;
  p_output1(5)=t5097;
  p_output1(6)=t6200;
  p_output1(7)=0.;
  p_output1(8)=t193*t369*t419 + t2696*t66;
  p_output1(9)=t1561*t369*t419 + t3358*t66;
  p_output1(10)=-1.*t1462*t419 + t3664*t66;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.183*t2989 - 0.13205*t4088 + t193*t369*t8750 + t1655*t8756 + t2459*t8798 + t2696*t8812 + var1(0);
  p_output1(13)=0. - 0.183*t3422 - 0.13205*t5097 + t1561*t369*t8750 + t3342*t8756 + t3356*t8798 + t3358*t8812 + var1(1);
  p_output1(14)=0. - 0.183*t3812 - 0.13205*t6200 - 1.*t1462*t8750 + t1292*t369*t8756 + t1583*t369*t8798 + t3664*t8812 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_RR_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_RR_thigh_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



