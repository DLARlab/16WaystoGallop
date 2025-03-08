/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:28 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FL_hip_joint_src.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static __inline__        /* use standard inline */
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
static void output1(double *p_output1,const double *var1)
{
  double t1535;
  double t1523;
  double t1572;
  double t1561;
  double t1583;
  double t1585;
  double t1582;
  double t1592;
  double t1584;
  double t1589;
  double t1590;
  double t1593;
  double t1594;
  double t1595;
  double t1598;
  double t1599;
  double t1600;
  double t1602;
  double t1603;
  double t1604;
  t1535 = Cos(var1[4]);
  t1523 = Cos(var1[3]);
  t1572 = Sin(var1[4]);
  t1561 = Sin(var1[3]);
  t1583 = Cos(var1[5]);
  t1585 = Sin(var1[5]);
  t1582 = Sin(var1[9]);
  t1592 = Cos(var1[9]);
  t1584 = t1523*t1583*t1572;
  t1589 = t1561*t1585;
  t1590 = t1584 + t1589;
  t1593 = -1.*t1583*t1561;
  t1594 = t1523*t1572*t1585;
  t1595 = t1593 + t1594;
  t1598 = t1583*t1561*t1572;
  t1599 = -1.*t1523*t1585;
  t1600 = t1598 + t1599;
  t1602 = t1523*t1583;
  t1603 = t1561*t1572*t1585;
  t1604 = t1602 + t1603;
  p_output1[0]=t1523*t1535;
  p_output1[1]=t1535*t1561;
  p_output1[2]=-1.*t1572;
  p_output1[3]=t1582*t1590 + t1592*t1595;
  p_output1[4]=t1582*t1600 + t1592*t1604;
  p_output1[5]=t1535*t1582*t1583 + t1535*t1585*t1592;
  p_output1[6]=t1590*t1592 - 1.*t1582*t1595;
  p_output1[7]=t1592*t1600 - 1.*t1582*t1604;
  p_output1[8]=-1.*t1535*t1582*t1585 + t1535*t1583*t1592;
}



void R_FL_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
