/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:43 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FR_hip_joint.h"

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
  double t1053;
  double t1152;
  double t902;
  double t1844;
  double t2667;
  double t2683;
  double t2662;
  double t7819;
  double t4473;
  double t7799;
  double t7809;
  double t2671;
  double t2687;
  double t2988;
  double t7828;
  double t7830;
  double t7831;
  double t7822;
  double t7824;
  double t7825;
  double t3150;
  double t7820;
  double t7821;
  double t7849;
  double t7850;
  double t7851;
  double t7852;
  double t7854;
  double t7855;
  double t7826;
  double t7832;
  double t7833;
  double t7834;
  double t7835;
  double t7836;
  t1053 = Cos(var1[4]);
  t1152 = Sin(var1[3]);
  t902 = Cos(var1[3]);
  t1844 = Sin(var1[4]);
  t2667 = Cos(var1[5]);
  t2683 = Sin(var1[5]);
  t2662 = Cos(var1[6]);
  t7819 = Sin(var1[6]);
  t4473 = t902*t2667*t1844;
  t7799 = t1152*t2683;
  t7809 = t4473 + t7799;
  t2671 = -1.*t2667*t1152;
  t2687 = t902*t1844*t2683;
  t2988 = t2671 + t2687;
  t7828 = t2667*t1152*t1844;
  t7830 = -1.*t902*t2683;
  t7831 = t7828 + t7830;
  t7822 = t902*t2667;
  t7824 = t1152*t1844*t2683;
  t7825 = t7822 + t7824;
  t3150 = t2662*t2988;
  t7820 = t7809*t7819;
  t7821 = t3150 + t7820;
  t7849 = -1.*t2662;
  t7850 = 1. + t7849;
  t7851 = -0.047*t7850;
  t7852 = 0. + t7851;
  t7854 = 0.047*t7819;
  t7855 = 0. + t7854;
  t7826 = t2662*t7825;
  t7832 = t7831*t7819;
  t7833 = t7826 + t7832;
  t7834 = t1053*t2662*t2683;
  t7835 = t1053*t2667*t7819;
  t7836 = t7834 + t7835;

  p_output1(0)=t1053*t902;
  p_output1(1)=t1053*t1152;
  p_output1(2)=-1.*t1844;
  p_output1(3)=0.;
  p_output1(4)=t7821;
  p_output1(5)=t7833;
  p_output1(6)=t7836;
  p_output1(7)=0.;
  p_output1(8)=t2662*t7809 - 1.*t2988*t7819;
  p_output1(9)=-1.*t7819*t7825 + t2662*t7831;
  p_output1(10)=t1053*t2662*t2667 - 1.*t1053*t2683*t7819;
  p_output1(11)=0.;
  p_output1(12)=0. - 0.047*t7821 + t2988*t7852 + t7809*t7855 + 0.183*t1053*t902 + var1(0);
  p_output1(13)=0. + 0.183*t1053*t1152 - 0.047*t7833 + t7825*t7852 + t7831*t7855 + var1(1);
  p_output1(14)=0. - 0.183*t1844 - 0.047*t7836 + t1053*t2683*t7852 + t1053*t2667*t7855 + var1(2);
  p_output1(15)=1.;
}


       
Eigen::Matrix<double,4,4> H_FR_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void H_FR_hip_joint(Eigen::Matrix<double,4,4> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,4,4>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



