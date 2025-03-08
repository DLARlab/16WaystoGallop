/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:07 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RR_hip_joint.h"

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
  double t1503;
  double t538;
  double t1576;
  double t1539;
  double t2340;
  double t2446;
  double t1889;
  double t4339;
  double t2344;
  double t2455;
  double t3113;
  double t4744;
  double t4990;
  double t5062;
  double t8636;
  double t8652;
  double t8662;
  double t8664;
  double t8665;
  double t8666;
  t1503 = Cos(var1[4]);
  t538 = Cos(var1[3]);
  t1576 = Sin(var1[4]);
  t1539 = Sin(var1[3]);
  t2340 = Cos(var1[5]);
  t2446 = Sin(var1[5]);
  t1889 = Sin(var1[12]);
  t4339 = Cos(var1[12]);
  t2344 = t538*t2340*t1576;
  t2455 = t1539*t2446;
  t3113 = t2344 + t2455;
  t4744 = -1.*t2340*t1539;
  t4990 = t538*t1576*t2446;
  t5062 = t4744 + t4990;
  t8636 = t2340*t1539*t1576;
  t8652 = -1.*t538*t2446;
  t8662 = t8636 + t8652;
  t8664 = t538*t2340;
  t8665 = t1539*t1576*t2446;
  t8666 = t8664 + t8665;

  p_output1(0)=t1503*t538;
  p_output1(1)=t1503*t1539;
  p_output1(2)=-1.*t1576;
  p_output1(3)=t1889*t3113 + t4339*t5062;
  p_output1(4)=t1889*t8662 + t4339*t8666;
  p_output1(5)=t1503*t1889*t2340 + t1503*t2446*t4339;
  p_output1(6)=t3113*t4339 - 1.*t1889*t5062;
  p_output1(7)=t4339*t8662 - 1.*t1889*t8666;
  p_output1(8)=-1.*t1503*t1889*t2446 + t1503*t2340*t4339;
}


       
Eigen::Matrix<double,3,3> R_RR_hip_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_RR_hip_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



