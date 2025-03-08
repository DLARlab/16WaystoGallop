/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:19 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RL_thigh_joint.h"

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
  double t2015;
  double t4495;
  double t5067;
  double t4510;
  double t6997;
  double t2739;
  double t4539;
  double t7420;
  double t7485;
  double t7667;
  double t8876;
  double t8896;
  double t8899;
  double t413;
  double t2016;
  double t8980;
  double t8999;
  double t9000;
  double t9002;
  double t948;
  double t1496;
  double t1510;
  double t1522;
  double t2883;
  double t3656;
  double t8013;
  double t8090;
  double t8756;
  double t8873;
  double t9025;
  double t9030;
  double t9033;
  double t9037;
  double t9041;
  double t9042;
  double t8981;
  double t8991;
  double t9050;
  double t9051;
  double t9052;
  double t9082;
  double t9084;
  double t9085;
  t2015 = Cos(var1[3]);
  t4495 = Cos(var1[5]);
  t5067 = Sin(var1[3]);
  t4510 = Sin(var1[4]);
  t6997 = Sin(var1[5]);
  t2739 = Sin(var1[15]);
  t4539 = t2015*t4495*t4510;
  t7420 = t5067*t6997;
  t7485 = t4539 + t7420;
  t7667 = Cos(var1[15]);
  t8876 = -1.*t4495*t5067;
  t8896 = t2015*t4510*t6997;
  t8899 = t8876 + t8896;
  t413 = Cos(var1[16]);
  t2016 = Cos(var1[4]);
  t8980 = Sin(var1[16]);
  t8999 = t7667*t7485;
  t9000 = -1.*t2739*t8899;
  t9002 = t8999 + t9000;
  t948 = -1.*t413;
  t1496 = 1. + t948;
  t1510 = -0.183*t1496;
  t1522 = 0. + t1510;
  t2883 = -0.047*t2739;
  t3656 = 0. + t2883;
  t8013 = -1.*t7667;
  t8090 = 1. + t8013;
  t8756 = 0.047*t8090;
  t8873 = 0. + t8756;
  t9025 = t4495*t5067*t4510;
  t9030 = -1.*t2015*t6997;
  t9033 = t9025 + t9030;
  t9037 = t2015*t4495;
  t9041 = t5067*t4510*t6997;
  t9042 = t9037 + t9041;
  t8981 = -0.183*t8980;
  t8991 = 0. + t8981;
  t9050 = t7667*t9033;
  t9051 = -1.*t2739*t9042;
  t9052 = t9050 + t9051;
  t9082 = t7667*t2016*t4495;
  t9084 = -1.*t2016*t2739*t6997;
  t9085 = t9082 + t9084;

  p_output1(0)=0. + t1522*t2015*t2016 + t3656*t7485 + t8873*t8899 + 0.13205*(t2739*t7485 + t7667*t8899) + t8991*t9002 - 0.183*(t2015*t2016*t413 - 1.*t8980*t9002) + var1(0);
  p_output1(1)=0. + t1522*t2016*t5067 + t3656*t9033 + t8873*t9042 + 0.13205*(t2739*t9033 + t7667*t9042) + t8991*t9052 - 0.183*(t2016*t413*t5067 - 1.*t8980*t9052) + var1(1);
  p_output1(2)=0. + t2016*t3656*t4495 - 1.*t1522*t4510 + 0.13205*(t2016*t2739*t4495 + t2016*t6997*t7667) + t2016*t6997*t8873 + t8991*t9085 - 0.183*(-1.*t413*t4510 - 1.*t8980*t9085) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_RL_thigh_joint(const Eigen::Matrix<double,18,1> &var1)
//void p_RL_thigh_joint(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



