/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:26 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RL_calf_joint.h"

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
  double t208;
  double t2478;
  double t3220;
  double t2831;
  double t3312;
  double t1126;
  double t373;
  double t403;
  double t1136;
  double t3149;
  double t3369;
  double t3604;
  double t3724;
  double t3904;
  double t3905;
  double t3911;
  double t4016;
  double t4132;
  double t4264;
  double t187;
  double t4379;
  double t4830;
  double t4881;
  double t4892;
  double t4957;
  double t4976;
  double t4980;
  double t4982;
  double t5064;
  double t5082;
  double t5248;
  double t5282;
  double t5340;
  double t792;
  double t4363;
  double t4369;
  double t4413;
  double t4427;
  double t4445;
  double t4718;
  double t5122;
  double t5123;
  double t5134;
  double t5136;
  double t5141;
  double t5172;
  double t5373;
  double t5414;
  double t5454;
  double t5512;
  double t5517;
  t208 = Cos(var1[3]);
  t2478 = Cos(var1[5]);
  t3220 = Sin(var1[3]);
  t2831 = Sin(var1[4]);
  t3312 = Sin(var1[5]);
  t1126 = Cos(var1[16]);
  t373 = Cos(var1[4]);
  t403 = Sin(var1[16]);
  t1136 = Cos(var1[15]);
  t3149 = t208*t2478*t2831;
  t3369 = t3220*t3312;
  t3604 = t3149 + t3369;
  t3724 = t1136*t3604;
  t3904 = Sin(var1[15]);
  t3905 = -1.*t2478*t3220;
  t3911 = t208*t2831*t3312;
  t4016 = t3905 + t3911;
  t4132 = -1.*t3904*t4016;
  t4264 = t3724 + t4132;
  t187 = Sin(var1[17]);
  t4379 = Cos(var1[17]);
  t4830 = t2478*t3220*t2831;
  t4881 = -1.*t208*t3312;
  t4892 = t4830 + t4881;
  t4957 = t1136*t4892;
  t4976 = t208*t2478;
  t4980 = t3220*t2831*t3312;
  t4982 = t4976 + t4980;
  t5064 = -1.*t3904*t4982;
  t5082 = t4957 + t5064;
  t5248 = t1136*t373*t2478;
  t5282 = -1.*t373*t3904*t3312;
  t5340 = t5248 + t5282;
  t792 = t208*t373*t403;
  t4363 = t1126*t4264;
  t4369 = t792 + t4363;
  t4413 = t1126*t208*t373;
  t4427 = -1.*t403*t4264;
  t4445 = t4413 + t4427;
  t4718 = t373*t403*t3220;
  t5122 = t1126*t5082;
  t5123 = t4718 + t5122;
  t5134 = t1126*t373*t3220;
  t5136 = -1.*t403*t5082;
  t5141 = t5134 + t5136;
  t5172 = -1.*t403*t2831;
  t5373 = t1126*t5340;
  t5414 = t5172 + t5373;
  t5454 = -1.*t1126*t2831;
  t5512 = -1.*t403*t5340;
  t5517 = t5454 + t5512;

  p_output1(0)=-1.*t187*t4369 + t4379*t4445;
  p_output1(1)=-1.*t187*t5123 + t4379*t5141;
  p_output1(2)=-1.*t187*t5414 + t4379*t5517;
  p_output1(3)=t3604*t3904 + t1136*t4016;
  p_output1(4)=t3904*t4892 + t1136*t4982;
  p_output1(5)=t1136*t3312*t373 + t2478*t373*t3904;
  p_output1(6)=t4369*t4379 + t187*t4445;
  p_output1(7)=t4379*t5123 + t187*t5141;
  p_output1(8)=t4379*t5414 + t187*t5517;
}


       
Eigen::Matrix<double,3,3> R_RL_calf_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_RL_calf_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



