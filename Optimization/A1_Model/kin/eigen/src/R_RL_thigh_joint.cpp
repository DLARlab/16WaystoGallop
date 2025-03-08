/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:22 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RL_thigh_joint.h"

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
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t751;
  double t2364;
  double t2964;
  double t2531;
  double t3009;
  double t407;
  double t808;
  double t1012;
  double t1013;
  double t3647;
  double t2590;
  double t3164;
  double t3302;
  double t3651;
  double t3878;
  double t4157;
  double t4488;
  double t4890;
  double t4893;
  double t4911;
  double t4912;
  double t5089;
  double t3618;
  double t4167;
  double t4309;
  double t4903;
  double t5091;
  double t5131;
  double t5401;
  double t5578;
  double t5796;
  t751 = Cos(var1[3]);
  t2364 = Cos(var1[5]);
  t2964 = Sin(var1[3]);
  t2531 = Sin(var1[4]);
  t3009 = Sin(var1[5]);
  t407 = Cos(var1[16]);
  t808 = Cos(var1[4]);
  t1012 = Sin(var1[16]);
  t1013 = Cos(var1[15]);
  t3647 = Sin(var1[15]);
  t2590 = t751*t2364*t2531;
  t3164 = t2964*t3009;
  t3302 = t2590 + t3164;
  t3651 = -1.*t2364*t2964;
  t3878 = t751*t2531*t3009;
  t4157 = t3651 + t3878;
  t4488 = t2364*t2964*t2531;
  t4890 = -1.*t751*t3009;
  t4893 = t4488 + t4890;
  t4911 = t751*t2364;
  t4912 = t2964*t2531*t3009;
  t5089 = t4911 + t4912;
  t3618 = t1013*t3302;
  t4167 = -1.*t3647*t4157;
  t4309 = t3618 + t4167;
  t4903 = t1013*t4893;
  t5091 = -1.*t3647*t5089;
  t5131 = t4903 + t5091;
  t5401 = t1013*t808*t2364;
  t5578 = -1.*t808*t3647*t3009;
  t5796 = t5401 + t5578;

  p_output1(0)=-1.*t1012*t4309 + t407*t751*t808;
  p_output1(1)=-1.*t1012*t5131 + t2964*t407*t808;
  p_output1(2)=-1.*t2531*t407 - 1.*t1012*t5796;
  p_output1(3)=t3302*t3647 + t1013*t4157;
  p_output1(4)=t3647*t4893 + t1013*t5089;
  p_output1(5)=t1013*t3009*t808 + t2364*t3647*t808;
  p_output1(6)=t407*t4309 + t1012*t751*t808;
  p_output1(7)=t407*t5131 + t1012*t2964*t808;
  p_output1(8)=-1.*t1012*t2531 + t407*t5796;
}


       
Eigen::Matrix<double,3,3> R_RL_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_RL_thigh_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



