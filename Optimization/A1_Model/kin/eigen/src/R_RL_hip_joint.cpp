/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:18 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RL_hip_joint.h"

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
  double t873;
  double t845;
  double t5504;
  double t3600;
  double t6800;
  double t6942;
  double t6791;
  double t6960;
  double t6898;
  double t6946;
  double t6947;
  double t6964;
  double t6972;
  double t6974;
  double t7147;
  double t7304;
  double t7335;
  double t7427;
  double t7466;
  double t7471;
  t873 = Cos(var1[4]);
  t845 = Cos(var1[3]);
  t5504 = Sin(var1[4]);
  t3600 = Sin(var1[3]);
  t6800 = Cos(var1[5]);
  t6942 = Sin(var1[5]);
  t6791 = Sin(var1[15]);
  t6960 = Cos(var1[15]);
  t6898 = t845*t6800*t5504;
  t6946 = t3600*t6942;
  t6947 = t6898 + t6946;
  t6964 = -1.*t6800*t3600;
  t6972 = t845*t5504*t6942;
  t6974 = t6964 + t6972;
  t7147 = t6800*t3600*t5504;
  t7304 = -1.*t845*t6942;
  t7335 = t7147 + t7304;
  t7427 = t845*t6800;
  t7466 = t3600*t5504*t6942;
  t7471 = t7427 + t7466;

  p_output1(0)=t845*t873;
  p_output1(1)=t3600*t873;
  p_output1(2)=-1.*t5504;
  p_output1(3)=t6791*t6947 + t6960*t6974;
  p_output1(4)=t6791*t7335 + t6960*t7471;
  p_output1(5)=t6791*t6800*t873 + t6942*t6960*t873;
  p_output1(6)=t6947*t6960 - 1.*t6791*t6974;
  p_output1(7)=t6960*t7335 - 1.*t6791*t7471;
  p_output1(8)=-1.*t6791*t6942*t873 + t6800*t6960*t873;
}


       
Eigen::Matrix<double,3,3> R_RL_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_RL_hip_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



