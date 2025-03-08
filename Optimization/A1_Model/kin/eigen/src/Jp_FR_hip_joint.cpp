/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:42 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FR_hip_joint.h"

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
  double t930;
  double t2639;
  double t2649;
  double t2634;
  double t2654;
  double t2597;
  double t2645;
  double t2655;
  double t2656;
  double t2661;
  double t2662;
  double t2667;
  double t2670;
  double t923;
  double t2603;
  double t2627;
  double t2628;
  double t2631;
  double t2671;
  double t2678;
  double t7808;
  double t7809;
  double t7811;
  double t7815;
  double t7819;
  double t7820;
  double t7855;
  double t7856;
  double t7857;
  double t7864;
  double t7865;
  double t7867;
  double t7859;
  double t7871;
  double t7894;
  double t7895;
  double t7896;
  double t7880;
  double t7881;
  double t7882;
  double t7883;
  t930 = Sin(var1[3]);
  t2639 = Cos(var1[5]);
  t2649 = Sin(var1[4]);
  t2634 = Cos(var1[3]);
  t2654 = Sin(var1[5]);
  t2597 = Cos(var1[6]);
  t2645 = -1.*t2634*t2639;
  t2655 = -1.*t930*t2649*t2654;
  t2656 = t2645 + t2655;
  t2661 = -1.*t2639*t930*t2649;
  t2662 = t2634*t2654;
  t2667 = t2661 + t2662;
  t2670 = Sin(var1[6]);
  t923 = Cos(var1[4]);
  t2603 = -1.*t2597;
  t2627 = 1. + t2603;
  t2628 = -0.047*t2627;
  t2631 = 0. + t2628;
  t2671 = 0.047*t2670;
  t2678 = 0. + t2671;
  t7808 = -1.*t2639*t930;
  t7809 = t2634*t2649*t2654;
  t7811 = t7808 + t7809;
  t7815 = t2634*t2639*t2649;
  t7819 = t930*t2654;
  t7820 = t7815 + t7819;
  t7855 = t2639*t930;
  t7856 = -1.*t2634*t2649*t2654;
  t7857 = t7855 + t7856;
  t7864 = t2639*t930*t2649;
  t7865 = -1.*t2634*t2654;
  t7867 = t7864 + t7865;
  t7859 = t2597*t7820;
  t7871 = t2597*t7867;
  t7894 = t2634*t2639;
  t7895 = t930*t2649*t2654;
  t7896 = t7894 + t7895;
  t7880 = t923*t2639*t2597;
  t7881 = -1.*t923*t2654*t2670;
  t7882 = t7880 + t7881;
  t7883 = -0.047*t7882;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t2631*t2656 - 0.047*(t2597*t2656 + t2667*t2670) + t2667*t2678 - 0.183*t923*t930;
  p_output1(10)=t2631*t7811 + t2678*t7820 - 0.047*(t2597*t7811 + t2670*t7820) + 0.183*t2634*t923;
  p_output1(11)=0;
  p_output1(12)=-0.183*t2634*t2649 + t2631*t2634*t2654*t923 + t2634*t2639*t2678*t923 - 0.047*(t2597*t2634*t2654*t923 + t2634*t2639*t2670*t923);
  p_output1(13)=-0.183*t2649*t930 + t2631*t2654*t923*t930 + t2639*t2678*t923*t930 - 0.047*(t2597*t2654*t923*t930 + t2639*t2670*t923*t930);
  p_output1(14)=-1.*t2631*t2649*t2654 - 0.047*(-1.*t2597*t2649*t2654 - 1.*t2639*t2649*t2670) - 1.*t2639*t2649*t2678 - 0.183*t923;
  p_output1(15)=t2631*t7820 + t2678*t7857 - 0.047*(t2670*t7857 + t7859);
  p_output1(16)=t2656*t2678 + t2631*t7867 - 0.047*(t2656*t2670 + t7871);
  p_output1(17)=t7883 + t2631*t2639*t923 - 1.*t2654*t2678*t923;
  p_output1(18)=-0.047*t2670*t7811 + 0.047*t2597*t7820 - 0.047*(-1.*t2670*t7811 + t7859);
  p_output1(19)=0.047*t2597*t7867 - 0.047*t2670*t7896 - 0.047*(t7871 - 1.*t2670*t7896);
  p_output1(20)=t7883 + 0.047*t2597*t2639*t923 - 0.047*t2654*t2670*t923;
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
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
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


       
Eigen::Matrix<double,3,18> Jp_FR_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void Jp_FR_hip_joint(Eigen::Matrix<double,3,18> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,18>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



