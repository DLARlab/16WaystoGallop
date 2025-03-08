/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:11 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RR_thigh_joint.h"

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
  double t871;
  double t1591;
  double t1606;
  double t1597;
  double t1607;
  double t474;
  double t946;
  double t1544;
  double t1571;
  double t1612;
  double t1605;
  double t1608;
  double t1609;
  double t1613;
  double t1614;
  double t1615;
  double t1624;
  double t1799;
  double t1879;
  double t3047;
  double t3413;
  double t3422;
  double t1610;
  double t1618;
  double t1619;
  double t2840;
  double t3425;
  double t5427;
  double t5782;
  double t6200;
  double t6460;
  t871 = Cos(var1[3]);
  t1591 = Cos(var1[5]);
  t1606 = Sin(var1[3]);
  t1597 = Sin(var1[4]);
  t1607 = Sin(var1[5]);
  t474 = Cos(var1[13]);
  t946 = Cos(var1[4]);
  t1544 = Sin(var1[13]);
  t1571 = Cos(var1[12]);
  t1612 = Sin(var1[12]);
  t1605 = t871*t1591*t1597;
  t1608 = t1606*t1607;
  t1609 = t1605 + t1608;
  t1613 = -1.*t1591*t1606;
  t1614 = t871*t1597*t1607;
  t1615 = t1613 + t1614;
  t1624 = t1591*t1606*t1597;
  t1799 = -1.*t871*t1607;
  t1879 = t1624 + t1799;
  t3047 = t871*t1591;
  t3413 = t1606*t1597*t1607;
  t3422 = t3047 + t3413;
  t1610 = t1571*t1609;
  t1618 = -1.*t1612*t1615;
  t1619 = t1610 + t1618;
  t2840 = t1571*t1879;
  t3425 = -1.*t1612*t3422;
  t5427 = t2840 + t3425;
  t5782 = t1571*t946*t1591;
  t6200 = -1.*t946*t1612*t1607;
  t6460 = t5782 + t6200;

  p_output1(0)=-1.*t1544*t1619 + t474*t871*t946;
  p_output1(1)=-1.*t1544*t5427 + t1606*t474*t946;
  p_output1(2)=-1.*t1597*t474 - 1.*t1544*t6460;
  p_output1(3)=t1609*t1612 + t1571*t1615;
  p_output1(4)=t1612*t1879 + t1571*t3422;
  p_output1(5)=t1571*t1607*t946 + t1591*t1612*t946;
  p_output1(6)=t1619*t474 + t1544*t871*t946;
  p_output1(7)=t474*t5427 + t1544*t1606*t946;
  p_output1(8)=-1.*t1544*t1597 + t474*t6460;
}


       
Eigen::Matrix<double,3,3> R_RR_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_RR_thigh_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



