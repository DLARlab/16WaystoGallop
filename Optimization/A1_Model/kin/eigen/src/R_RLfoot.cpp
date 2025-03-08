/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:41 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RLfoot.h"

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
  double t2976;
  double t4113;
  double t4337;
  double t4302;
  double t4437;
  double t3832;
  double t3303;
  double t3336;
  double t4101;
  double t4325;
  double t4491;
  double t4492;
  double t4683;
  double t4792;
  double t4870;
  double t4872;
  double t4919;
  double t4938;
  double t4977;
  double t374;
  double t5179;
  double t5538;
  double t5539;
  double t5542;
  double t5610;
  double t5646;
  double t5648;
  double t5662;
  double t5671;
  double t5716;
  double t5839;
  double t5840;
  double t5841;
  double t3371;
  double t4985;
  double t5142;
  double t5325;
  double t5419;
  double t5430;
  double t5507;
  double t5756;
  double t5778;
  double t5788;
  double t5795;
  double t5802;
  double t5833;
  double t5844;
  double t5860;
  double t5873;
  double t5876;
  double t5881;
  t2976 = Cos(var1[3]);
  t4113 = Cos(var1[5]);
  t4337 = Sin(var1[3]);
  t4302 = Sin(var1[4]);
  t4437 = Sin(var1[5]);
  t3832 = Cos(var1[16]);
  t3303 = Cos(var1[4]);
  t3336 = Sin(var1[16]);
  t4101 = Cos(var1[15]);
  t4325 = t2976*t4113*t4302;
  t4491 = t4337*t4437;
  t4492 = t4325 + t4491;
  t4683 = t4101*t4492;
  t4792 = Sin(var1[15]);
  t4870 = -1.*t4113*t4337;
  t4872 = t2976*t4302*t4437;
  t4919 = t4870 + t4872;
  t4938 = -1.*t4792*t4919;
  t4977 = t4683 + t4938;
  t374 = Sin(var1[17]);
  t5179 = Cos(var1[17]);
  t5538 = t4113*t4337*t4302;
  t5539 = -1.*t2976*t4437;
  t5542 = t5538 + t5539;
  t5610 = t4101*t5542;
  t5646 = t2976*t4113;
  t5648 = t4337*t4302*t4437;
  t5662 = t5646 + t5648;
  t5671 = -1.*t4792*t5662;
  t5716 = t5610 + t5671;
  t5839 = t4101*t3303*t4113;
  t5840 = -1.*t3303*t4792*t4437;
  t5841 = t5839 + t5840;
  t3371 = t2976*t3303*t3336;
  t4985 = t3832*t4977;
  t5142 = t3371 + t4985;
  t5325 = t3832*t2976*t3303;
  t5419 = -1.*t3336*t4977;
  t5430 = t5325 + t5419;
  t5507 = t3303*t3336*t4337;
  t5756 = t3832*t5716;
  t5778 = t5507 + t5756;
  t5788 = t3832*t3303*t4337;
  t5795 = -1.*t3336*t5716;
  t5802 = t5788 + t5795;
  t5833 = -1.*t3336*t4302;
  t5844 = t3832*t5841;
  t5860 = t5833 + t5844;
  t5873 = -1.*t3832*t4302;
  t5876 = -1.*t3336*t5841;
  t5881 = t5873 + t5876;

  p_output1(0)=-1.*t374*t5142 + t5179*t5430;
  p_output1(1)=-1.*t374*t5778 + t5179*t5802;
  p_output1(2)=-1.*t374*t5860 + t5179*t5881;
  p_output1(3)=t4492*t4792 + t4101*t4919;
  p_output1(4)=t4792*t5542 + t4101*t5662;
  p_output1(5)=t3303*t4101*t4437 + t3303*t4113*t4792;
  p_output1(6)=t5142*t5179 + t374*t5430;
  p_output1(7)=t5179*t5778 + t374*t5802;
  p_output1(8)=t5179*t5860 + t374*t5881;
}


       
Eigen::Matrix<double,3,3> R_RLfoot(const Eigen::Matrix<double,18,1> &var1)
//void R_RLfoot(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



