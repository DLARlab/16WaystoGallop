/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:48 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FR_thigh_joint.h"

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
  double t256;
  double t1952;
  double t2618;
  double t2328;
  double t2632;
  double t386;
  double t672;
  double t1950;
  double t3696;
  double t4455;
  double t2668;
  double t2673;
  double t3099;
  double t2557;
  double t2646;
  double t2651;
  double t7965;
  double t7968;
  double t7969;
  double t7943;
  double t7944;
  double t7954;
  double t2659;
  double t3920;
  double t4347;
  double t7956;
  double t7972;
  double t7974;
  double t7980;
  double t7985;
  double t7986;
  t256 = Cos(var1[3]);
  t1952 = Cos(var1[5]);
  t2618 = Sin(var1[3]);
  t2328 = Sin(var1[4]);
  t2632 = Sin(var1[5]);
  t386 = Cos(var1[4]);
  t672 = Cos(var1[7]);
  t1950 = Cos(var1[6]);
  t3696 = Sin(var1[6]);
  t4455 = Sin(var1[7]);
  t2668 = -1.*t1952*t2618;
  t2673 = t256*t2328*t2632;
  t3099 = t2668 + t2673;
  t2557 = t256*t1952*t2328;
  t2646 = t2618*t2632;
  t2651 = t2557 + t2646;
  t7965 = t256*t1952;
  t7968 = t2618*t2328*t2632;
  t7969 = t7965 + t7968;
  t7943 = t1952*t2618*t2328;
  t7944 = -1.*t256*t2632;
  t7954 = t7943 + t7944;
  t2659 = t1950*t2651;
  t3920 = -1.*t3099*t3696;
  t4347 = t2659 + t3920;
  t7956 = t1950*t7954;
  t7972 = -1.*t7969*t3696;
  t7974 = t7956 + t7972;
  t7980 = t386*t1952*t1950;
  t7985 = -1.*t386*t2632*t3696;
  t7986 = t7980 + t7985;

  p_output1(0)=-1.*t4347*t4455 + t256*t386*t672;
  p_output1(1)=t2618*t386*t672 - 1.*t4455*t7974;
  p_output1(2)=-1.*t2328*t672 - 1.*t4455*t7986;
  p_output1(3)=t1950*t3099 + t2651*t3696;
  p_output1(4)=t3696*t7954 + t1950*t7969;
  p_output1(5)=t1950*t2632*t386 + t1952*t3696*t386;
  p_output1(6)=t256*t386*t4455 + t4347*t672;
  p_output1(7)=t2618*t386*t4455 + t672*t7974;
  p_output1(8)=-1.*t2328*t4455 + t672*t7986;
}


       
Eigen::Matrix<double,3,3> R_FR_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void R_FR_thigh_joint(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,3>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



