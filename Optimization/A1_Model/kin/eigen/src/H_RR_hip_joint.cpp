/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:06 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_RR_hip_joint.h"

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
  double t322;
  double t158;
  double t2783;
  double t1878;
  double t4159;
  double t4997;
  double t3882;
  double t7072;
  double t4418;
  double t5636;
  double t6074;
  double t8575;
  double t8594;
  double t8595;
  double t8603;
  double t8605;
  double t8608;
  double t8617;
  double t8622;
  double t8630;
  double t6221;
  double t8598;
  double t8602;
  double t8670;
  double t8672;
  double t8678;
  double t8680;
  double t8682;
  double t8684;
  double t8611;
  double t8631;
  double t8636;
  double t8639;
  double t8643;
  double t8650;
  t322 = Cos(var1[4]);
  t158 = Cos(var1[3]);
  t2783 = Sin(var1[4]);
  t1878 = Sin(var1[3]);
  t4159 = Cos(var1[5]);
  t4997 = Sin(var1[5]);
  t3882 = Sin(var1[12]);
  t7072 = Cos(var1[12]);
  t4418 = t158*t4159*t2783;
  t5636 = t1878*t4997;
  t6074 = t4418 + t5636;
  t8575 = -1.*t4159*t1878;
  t8594 = t158*t2783*t4997;
  t8595 = t8575 + t8594;
  t8603 = t4159*t1878*t2783;
  t8605 = -1.*t158*t4997;
  t8608 = t8603 + t8605;
  t8617 = t158*t4159;
  t8622 = t1878*t2783*t4997;
  t8630 = t8617 + t8622;
  t6221 = t3882*t6074;
  t8598 = t7072*t8595;
  t8602 = t6221 + t8598;
  t8670 = 0.047*t3882;
  t8672 = 0. + t8670;
  t8678 = -1.*t7072;
  t8680 = 1. + t8678;
  t8682 = -0.047*t8680;
  t8684 = 0. + t8682;
  t8611 = t3882*t8608;
  t8631 = t7072*t8630;
  t8636 = t8611 + t8631;
  t8639 = t322*t4159*t3882;
  t8643 = t7072*t322*t4997;
  t8650 = t8639 + t8643;

  p_output1(0)=t158*t322;
  p_output1(1)=t1878*t322;
  p_output1(2)=-1.*t2783;
  p_output1(3)=0.;
  p_output1(4)=t8602;
  p_output1(5)=t8636;
  p_output1(6)=t8650;
  p_output1(7)=0.;
  p_output1(8)=t6074*t7072 - 1.*t3882*t8595;
  p_output1(9)=t7072*t8608 - 1.*t3882*t8630;
  p_output1(10)=-1.*t322*t3882*t4997 + t322*t4159*t7072;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.183*t158*t322 - 0.047*t8602 + t6074*t8672 + t8595*t8684 + var1(0);
  p_output1(13)=0. - 0.183*t1878*t322 - 0.047*t8636 + t8608*t8672 + t8630*t8684 + var1(1);
  p_output1(14)=0. + 0.183*t2783 - 0.047*t8650 + t322*t4159*t8672 + t322*t4997*t8684 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_RR_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_RR_hip_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



