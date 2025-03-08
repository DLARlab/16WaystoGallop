/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:48 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RL_hip_joint_src.h"

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
  double t4048;
  double t4082;
  double t4087;
  double t4085;
  double t4091;
  double t4069;
  double t4086;
  double t4092;
  double t4094;
  double t4096;
  double t4101;
  double t4102;
  double t4103;
  double t4049;
  double t4070;
  double t4081;
  double t4097;
  double t4098;
  double t4099;
  double t4100;
  double t4112;
  double t4113;
  double t4114;
  double t4116;
  double t4117;
  double t4118;
  t4048 = Cos(var1[3]);
  t4082 = Cos(var1[5]);
  t4087 = Sin(var1[3]);
  t4085 = Sin(var1[4]);
  t4091 = Sin(var1[5]);
  t4069 = Sin(var1[15]);
  t4086 = t4048*t4082*t4085;
  t4092 = t4087*t4091;
  t4094 = t4086 + t4092;
  t4096 = Cos(var1[15]);
  t4101 = -1.*t4082*t4087;
  t4102 = t4048*t4085*t4091;
  t4103 = t4101 + t4102;
  t4049 = Cos(var1[4]);
  t4070 = -0.047*t4069;
  t4081 = 0. + t4070;
  t4097 = -1.*t4096;
  t4098 = 1. + t4097;
  t4099 = 0.047*t4098;
  t4100 = 0. + t4099;
  t4112 = t4082*t4087*t4085;
  t4113 = -1.*t4048*t4091;
  t4114 = t4112 + t4113;
  t4116 = t4048*t4082;
  t4117 = t4087*t4085*t4091;
  t4118 = t4116 + t4117;
  p_output1[0]=0. - 0.183*t4048*t4049 + t4081*t4094 + t4100*t4103 + 0.047*(t4069*t4094 + t4096*t4103) + var1[0];
  p_output1[1]=0. - 0.183*t4049*t4087 + t4081*t4114 + t4100*t4118 + 0.047*(t4069*t4114 + t4096*t4118) + var1[1];
  p_output1[2]=0. + t4049*t4081*t4082 + 0.183*t4085 + 0.047*(t4049*t4069*t4082 + t4049*t4091*t4096) + t4049*t4091*t4100 + var1[2];
}



void p_RL_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
