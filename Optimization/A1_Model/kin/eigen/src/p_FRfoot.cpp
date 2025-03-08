/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:27 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FRfoot.h"

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
  double t389;
  double t2629;
  double t3590;
  double t2638;
  double t3725;
  double t1797;
  double t2656;
  double t3795;
  double t3846;
  double t3857;
  double t3859;
  double t3952;
  double t4606;
  double t712;
  double t5579;
  double t5584;
  double t5595;
  double t433;
  double t5602;
  double t5711;
  double t5723;
  double t5799;
  double t5880;
  double t6095;
  double t6097;
  double t6110;
  double t5691;
  double t5703;
  double t5704;
  double t997;
  double t1005;
  double t1056;
  double t1738;
  double t1807;
  double t1812;
  double t2371;
  double t2626;
  double t4902;
  double t4930;
  double t6661;
  double t6687;
  double t6727;
  double t6931;
  double t6943;
  double t7268;
  double t5612;
  double t5629;
  double t7604;
  double t7666;
  double t7673;
  double t5879;
  double t5887;
  double t5930;
  double t6131;
  double t6253;
  double t6257;
  double t8097;
  double t8119;
  double t8332;
  double t7743;
  double t7747;
  double t7869;
  double t9215;
  double t9228;
  double t9244;
  double t9291;
  double t9298;
  double t9300;
  double t9267;
  double t9273;
  double t9276;
  t389 = Cos(var1[3]);
  t2629 = Cos(var1[5]);
  t3590 = Sin(var1[4]);
  t2638 = Sin(var1[3]);
  t3725 = Sin(var1[5]);
  t1797 = Cos(var1[6]);
  t2656 = -1.*t2629*t2638;
  t3795 = t389*t3590*t3725;
  t3846 = t2656 + t3795;
  t3857 = t389*t2629*t3590;
  t3859 = t2638*t3725;
  t3952 = t3857 + t3859;
  t4606 = Sin(var1[6]);
  t712 = Cos(var1[7]);
  t5579 = t1797*t3952;
  t5584 = -1.*t3846*t4606;
  t5595 = t5579 + t5584;
  t433 = Cos(var1[4]);
  t5602 = Sin(var1[7]);
  t5711 = Cos(var1[8]);
  t5723 = -1.*t5711;
  t5799 = 1. + t5723;
  t5880 = Sin(var1[8]);
  t6095 = t389*t433*t712;
  t6097 = -1.*t5595*t5602;
  t6110 = t6095 + t6097;
  t5691 = t712*t5595;
  t5703 = t389*t433*t5602;
  t5704 = t5691 + t5703;
  t997 = -1.*t712;
  t1005 = 1. + t997;
  t1056 = 0.183*t1005;
  t1738 = 0. + t1056;
  t1807 = -1.*t1797;
  t1812 = 1. + t1807;
  t2371 = -0.047*t1812;
  t2626 = 0. + t2371;
  t4902 = 0.047*t4606;
  t4930 = 0. + t4902;
  t6661 = t389*t2629;
  t6687 = t2638*t3590*t3725;
  t6727 = t6661 + t6687;
  t6931 = t2629*t2638*t3590;
  t6943 = -1.*t389*t3725;
  t7268 = t6931 + t6943;
  t5612 = 0.183*t5602;
  t5629 = 0. + t5612;
  t7604 = t1797*t7268;
  t7666 = -1.*t6727*t4606;
  t7673 = t7604 + t7666;
  t5879 = -0.2*t5799;
  t5887 = 0.183*t5880;
  t5930 = 0. + t5879 + t5887;
  t6131 = 0.183*t5799;
  t6253 = 0.2*t5880;
  t6257 = 0. + t6131 + t6253;
  t8097 = t433*t712*t2638;
  t8119 = -1.*t7673*t5602;
  t8332 = t8097 + t8119;
  t7743 = t712*t7673;
  t7747 = t433*t2638*t5602;
  t7869 = t7743 + t7747;
  t9215 = t433*t2629*t1797;
  t9228 = -1.*t433*t3725*t4606;
  t9244 = t9215 + t9228;
  t9291 = -1.*t712*t3590;
  t9298 = -1.*t9244*t5602;
  t9300 = t9291 + t9298;
  t9267 = t712*t9244;
  t9273 = -1.*t3590*t5602;
  t9276 = t9267 + t9273;

  p_output1(0)=0. + t2626*t3846 + t1738*t389*t433 - 0.13205*(t1797*t3846 + t3952*t4606) + t3952*t4930 + t5595*t5629 + t5704*t5930 + 0.183*(-1.*t5704*t5880 + t5711*t6110) - 0.4*(t5704*t5711 + t5880*t6110) + t6110*t6257 + var1(0);
  p_output1(1)=0. + t1738*t2638*t433 + t2626*t6727 + t4930*t7268 - 0.13205*(t1797*t6727 + t4606*t7268) + t5629*t7673 + t5930*t7869 + t6257*t8332 + 0.183*(-1.*t5880*t7869 + t5711*t8332) - 0.4*(t5711*t7869 + t5880*t8332) + var1(1);
  p_output1(2)=0. - 1.*t1738*t3590 + t2626*t3725*t433 - 0.13205*(t1797*t3725*t433 + t2629*t433*t4606) + t2629*t433*t4930 + t5629*t9244 + t5930*t9276 + t6257*t9300 + 0.183*(-1.*t5880*t9276 + t5711*t9300) - 0.4*(t5711*t9276 + t5880*t9300) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_FRfoot(const Eigen::Matrix<double,18,1> &var1)
//void p_FRfoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



