/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:21 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RL_thigh_joint.h"

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
  double t207;
  double t266;
  double t270;
  double t267;
  double t271;
  double t167;
  double t250;
  double t254;
  double t258;
  double t647;
  double t268;
  double t272;
  double t448;
  double t720;
  double t882;
  double t990;
  double t1734;
  double t1737;
  double t1743;
  double t2201;
  double t2364;
  double t2427;
  double t596;
  double t994;
  double t1510;
  double t1868;
  double t2432;
  double t2439;
  double t2765;
  double t2961;
  double t2974;
  double t3134;
  double t3139;
  double t3170;
  double t253;
  double t1665;
  double t1666;
  double t4915;
  double t5025;
  double t5178;
  double t5252;
  double t5318;
  double t5401;
  double t5705;
  double t5714;
  double t5831;
  double t6386;
  double t3225;
  double t3255;
  double t3396;
  double t7142;
  double t7149;
  double t1668;
  double t2475;
  double t2531;
  double t3647;
  double t3651;
  double t3695;
  double t2738;
  double t2992;
  double t3010;
  t207 = Cos(var1[3]);
  t266 = Cos(var1[5]);
  t270 = Sin(var1[3]);
  t267 = Sin(var1[4]);
  t271 = Sin(var1[5]);
  t167 = Cos(var1[16]);
  t250 = Cos(var1[4]);
  t254 = Sin(var1[16]);
  t258 = Cos(var1[15]);
  t647 = Sin(var1[15]);
  t268 = t207*t266*t267;
  t272 = t270*t271;
  t448 = t268 + t272;
  t720 = -1.*t266*t270;
  t882 = t207*t267*t271;
  t990 = t720 + t882;
  t1734 = t266*t270*t267;
  t1737 = -1.*t207*t271;
  t1743 = t1734 + t1737;
  t2201 = t207*t266;
  t2364 = t270*t267*t271;
  t2427 = t2201 + t2364;
  t596 = t258*t448;
  t994 = -1.*t647*t990;
  t1510 = t596 + t994;
  t1868 = t258*t1743;
  t2432 = -1.*t647*t2427;
  t2439 = t1868 + t2432;
  t2765 = t258*t250*t266;
  t2961 = -1.*t250*t647*t271;
  t2974 = t2765 + t2961;
  t3134 = t647*t448;
  t3139 = t258*t990;
  t3170 = t3134 + t3139;
  t253 = t167*t207*t250;
  t1665 = -1.*t254*t1510;
  t1666 = t253 + t1665;
  t4915 = -1.*t167;
  t5025 = 1. + t4915;
  t5178 = -0.183*t5025;
  t5252 = 0. + t5178;
  t5318 = -0.047*t647;
  t5401 = 0. + t5318;
  t5705 = -1.*t258;
  t5714 = 1. + t5705;
  t5831 = 0.047*t5714;
  t6386 = 0. + t5831;
  t3225 = t647*t1743;
  t3255 = t258*t2427;
  t3396 = t3225 + t3255;
  t7142 = -0.183*t254;
  t7149 = 0. + t7142;
  t1668 = t167*t250*t270;
  t2475 = -1.*t254*t2439;
  t2531 = t1668 + t2475;
  t3647 = t250*t266*t647;
  t3651 = t258*t250*t271;
  t3695 = t3647 + t3651;
  t2738 = -1.*t167*t267;
  t2992 = -1.*t254*t2974;
  t3010 = t2738 + t2992;

  p_output1(0)=t1666;
  p_output1(1)=t2531;
  p_output1(2)=t3010;
  p_output1(3)=0.;
  p_output1(4)=t3170;
  p_output1(5)=t3396;
  p_output1(6)=t3695;
  p_output1(7)=0.;
  p_output1(8)=t1510*t167 + t207*t250*t254;
  p_output1(9)=t167*t2439 + t250*t254*t270;
  p_output1(10)=-1.*t254*t267 + t167*t2974;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.183*t1666 + 0.13205*t3170 + t207*t250*t5252 + t448*t5401 + t1510*t7149 + t6386*t990 + var1(0);
  p_output1(13)=0. - 0.183*t2531 + 0.13205*t3396 + t250*t270*t5252 + t1743*t5401 + t2427*t6386 + t2439*t7149 + var1(1);
  p_output1(14)=0. - 0.183*t3010 + 0.13205*t3695 - 1.*t267*t5252 + t250*t266*t5401 + t250*t271*t6386 + t2974*t7149 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_RL_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_RL_thigh_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



