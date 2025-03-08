/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:05 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_RR_hip_joint.h"

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
static void output1(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t31;
  double t1927;
  double t888;
  double t908;
  double t3706;
  double t36;
  double t1413;
  double t3882;
  double t4156;
  double t5468;
  double t6061;
  double t6064;
  double t6074;
  double t29;
  double t322;
  double t385;
  double t5489;
  double t5583;
  double t5636;
  double t5767;
  double t8598;
  double t8602;
  double t8603;
  double t8607;
  double t8608;
  double t8609;
  double t8692;
  double t8693;
  double t8694;
  double t8711;
  double t8715;
  double t8716;
  double t8700;
  double t8720;
  double t8756;
  double t8757;
  double t8758;
  double t8736;
  double t8737;
  double t8740;
  double t8741;
  t31 = Sin(var1[3]);
  t1927 = Cos(var1[3]);
  t888 = Cos(var1[5]);
  t908 = Sin(var1[4]);
  t3706 = Sin(var1[5]);
  t36 = Sin(var1[12]);
  t1413 = -1.*t888*t31*t908;
  t3882 = t1927*t3706;
  t4156 = t1413 + t3882;
  t5468 = Cos(var1[12]);
  t6061 = -1.*t1927*t888;
  t6064 = -1.*t31*t908*t3706;
  t6074 = t6061 + t6064;
  t29 = Cos(var1[4]);
  t322 = 0.047*t36;
  t385 = 0. + t322;
  t5489 = -1.*t5468;
  t5583 = 1. + t5489;
  t5636 = -0.047*t5583;
  t5767 = 0. + t5636;
  t8598 = t1927*t888*t908;
  t8602 = t31*t3706;
  t8603 = t8598 + t8602;
  t8607 = -1.*t888*t31;
  t8608 = t1927*t908*t3706;
  t8609 = t8607 + t8608;
  t8692 = t888*t31;
  t8693 = -1.*t1927*t908*t3706;
  t8694 = t8692 + t8693;
  t8711 = t888*t31*t908;
  t8715 = -1.*t1927*t3706;
  t8716 = t8711 + t8715;
  t8700 = t5468*t8603;
  t8720 = t5468*t8716;
  t8756 = t1927*t888;
  t8757 = t31*t908*t3706;
  t8758 = t8756 + t8757;
  t8736 = t5468*t29*t888;
  t8737 = -1.*t29*t36*t3706;
  t8740 = t8736 + t8737;
  t8741 = -0.047*t8740;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=0.183*t29*t31 + t385*t4156 + t5767*t6074 - 0.047*(t36*t4156 + t5468*t6074);
  p_output1(10)=-0.183*t1927*t29 + t385*t8603 + t5767*t8609 - 0.047*(t36*t8603 + t5468*t8609);
  p_output1(11)=0;
  p_output1(12)=t1927*t29*t3706*t5767 + t1927*t29*t385*t888 - 0.047*(t1927*t29*t3706*t5468 + t1927*t29*t36*t888) + 0.183*t1927*t908;
  p_output1(13)=t29*t31*t3706*t5767 + t29*t31*t385*t888 - 0.047*(t29*t31*t3706*t5468 + t29*t31*t36*t888) + 0.183*t31*t908;
  p_output1(14)=0.183*t29 - 1.*t3706*t5767*t908 - 1.*t385*t888*t908 - 0.047*(-1.*t3706*t5468*t908 - 1.*t36*t888*t908);
  p_output1(15)=t5767*t8603 + t385*t8694 - 0.047*(t36*t8694 + t8700);
  p_output1(16)=t385*t6074 + t5767*t8716 - 0.047*(t36*t6074 + t8720);
  p_output1(17)=-1.*t29*t3706*t385 + t8741 + t29*t5767*t888;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
  p_output1(36)=0.047*t5468*t8603 - 0.047*t36*t8609 - 0.047*(-1.*t36*t8609 + t8700);
  p_output1(37)=0.047*t5468*t8716 - 0.047*t36*t8758 - 0.047*(t8720 - 1.*t36*t8758);
  p_output1(38)=-0.047*t29*t36*t3706 + t8741 + 0.047*t29*t5468*t888;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
  p_output1(48)=0;
  p_output1(49)=0;
  p_output1(50)=0;
  p_output1(51)=0;
  p_output1(52)=0;
  p_output1(53)=0;
}


       
Eigen::Matrix<double,3,18> Jp_RR_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_RR_hip_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



