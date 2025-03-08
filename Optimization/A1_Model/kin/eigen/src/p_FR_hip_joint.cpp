/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:41 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FR_hip_joint.h"

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
  double t3798;
  double t7793;
  double t7796;
  double t7794;
  double t7797;
  double t7782;
  double t7795;
  double t7798;
  double t7799;
  double t7801;
  double t7802;
  double t7803;
  double t7804;
  double t7757;
  double t7787;
  double t7790;
  double t7791;
  double t7792;
  double t7805;
  double t7807;
  double t7817;
  double t7819;
  double t7820;
  double t7822;
  double t7823;
  double t7824;
  t3798 = Cos(var1[3]);
  t7793 = Cos(var1[5]);
  t7796 = Sin(var1[4]);
  t7794 = Sin(var1[3]);
  t7797 = Sin(var1[5]);
  t7782 = Cos(var1[6]);
  t7795 = -1.*t7793*t7794;
  t7798 = t3798*t7796*t7797;
  t7799 = t7795 + t7798;
  t7801 = t3798*t7793*t7796;
  t7802 = t7794*t7797;
  t7803 = t7801 + t7802;
  t7804 = Sin(var1[6]);
  t7757 = Cos(var1[4]);
  t7787 = -1.*t7782;
  t7790 = 1. + t7787;
  t7791 = -0.047*t7790;
  t7792 = 0. + t7791;
  t7805 = 0.047*t7804;
  t7807 = 0. + t7805;
  t7817 = t3798*t7793;
  t7819 = t7794*t7796*t7797;
  t7820 = t7817 + t7819;
  t7822 = t7793*t7794*t7796;
  t7823 = -1.*t3798*t7797;
  t7824 = t7822 + t7823;

  p_output1(0)=0. + 0.183*t3798*t7757 + t7792*t7799 - 0.047*(t7782*t7799 + t7803*t7804) + t7803*t7807 + var1(0);
  p_output1(1)=0. + 0.183*t7757*t7794 + t7792*t7820 + t7807*t7824 - 0.047*(t7782*t7820 + t7804*t7824) + var1(1);
  p_output1(2)=0. - 0.183*t7796 + t7757*t7792*t7797 - 0.047*(t7757*t7782*t7797 + t7757*t7793*t7804) + t7757*t7793*t7807 + var1(2);
}


       
Eigen::Matrix<double,3,1> p_FR_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_FR_hip_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



