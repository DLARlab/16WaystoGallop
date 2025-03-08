/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:47 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FR_thigh_joint.h"

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
  double t28;
  double t831;
  double t964;
  double t885;
  double t1120;
  double t148;
  double t479;
  double t594;
  double t3936;
  double t4200;
  double t3584;
  double t3696;
  double t3702;
  double t940;
  double t1258;
  double t1684;
  double t7943;
  double t7944;
  double t7946;
  double t7932;
  double t7933;
  double t7934;
  double t3022;
  double t3940;
  double t4134;
  double t7938;
  double t7948;
  double t7949;
  double t7956;
  double t7960;
  double t7961;
  double t7968;
  double t7969;
  double t7972;
  double t591;
  double t4561;
  double t7894;
  double t7995;
  double t7996;
  double t7997;
  double t8000;
  double t8004;
  double t8005;
  double t8006;
  double t8007;
  double t8009;
  double t8010;
  double t7974;
  double t7977;
  double t7978;
  double t8014;
  double t8016;
  double t7918;
  double t7950;
  double t7954;
  double t7979;
  double t7980;
  double t7985;
  double t7955;
  double t7962;
  double t7965;
  t28 = Cos(var1[3]);
  t831 = Cos(var1[5]);
  t964 = Sin(var1[3]);
  t885 = Sin(var1[4]);
  t1120 = Sin(var1[5]);
  t148 = Cos(var1[4]);
  t479 = Cos(var1[7]);
  t594 = Cos(var1[6]);
  t3936 = Sin(var1[6]);
  t4200 = Sin(var1[7]);
  t3584 = -1.*t831*t964;
  t3696 = t28*t885*t1120;
  t3702 = t3584 + t3696;
  t940 = t28*t831*t885;
  t1258 = t964*t1120;
  t1684 = t940 + t1258;
  t7943 = t28*t831;
  t7944 = t964*t885*t1120;
  t7946 = t7943 + t7944;
  t7932 = t831*t964*t885;
  t7933 = -1.*t28*t1120;
  t7934 = t7932 + t7933;
  t3022 = t594*t1684;
  t3940 = -1.*t3702*t3936;
  t4134 = t3022 + t3940;
  t7938 = t594*t7934;
  t7948 = -1.*t7946*t3936;
  t7949 = t7938 + t7948;
  t7956 = t148*t831*t594;
  t7960 = -1.*t148*t1120*t3936;
  t7961 = t7956 + t7960;
  t7968 = t594*t3702;
  t7969 = t1684*t3936;
  t7972 = t7968 + t7969;
  t591 = t28*t148*t479;
  t4561 = -1.*t4134*t4200;
  t7894 = t591 + t4561;
  t7995 = -1.*t479;
  t7996 = 1. + t7995;
  t7997 = 0.183*t7996;
  t8000 = 0. + t7997;
  t8004 = -1.*t594;
  t8005 = 1. + t8004;
  t8006 = -0.047*t8005;
  t8007 = 0. + t8006;
  t8009 = 0.047*t3936;
  t8010 = 0. + t8009;
  t7974 = t594*t7946;
  t7977 = t7934*t3936;
  t7978 = t7974 + t7977;
  t8014 = 0.183*t4200;
  t8016 = 0. + t8014;
  t7918 = t148*t479*t964;
  t7950 = -1.*t7949*t4200;
  t7954 = t7918 + t7950;
  t7979 = t148*t594*t1120;
  t7980 = t148*t831*t3936;
  t7985 = t7979 + t7980;
  t7955 = -1.*t479*t885;
  t7962 = -1.*t7961*t4200;
  t7965 = t7955 + t7962;

  p_output1(0)=t7894;
  p_output1(1)=t7954;
  p_output1(2)=t7965;
  p_output1(3)=0.;
  p_output1(4)=t7972;
  p_output1(5)=t7978;
  p_output1(6)=t7985;
  p_output1(7)=0.;
  p_output1(8)=t148*t28*t4200 + t4134*t479;
  p_output1(9)=t479*t7949 + t148*t4200*t964;
  p_output1(10)=t479*t7961 - 1.*t4200*t885;
  p_output1(11)=0.;
  p_output1(12)=0. + 0.183*t7894 - 0.13205*t7972 + t148*t28*t8000 + t3702*t8007 + t1684*t8010 + t4134*t8016 + var1(0);
  p_output1(13)=0. + 0.183*t7954 - 0.13205*t7978 + t7946*t8007 + t7934*t8010 + t7949*t8016 + t148*t8000*t964 + var1(1);
  p_output1(14)=0. + 0.183*t7965 - 0.13205*t7985 + t1120*t148*t8007 + t7961*t8016 + t148*t8010*t831 - 1.*t8000*t885 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_FR_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_FR_thigh_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



