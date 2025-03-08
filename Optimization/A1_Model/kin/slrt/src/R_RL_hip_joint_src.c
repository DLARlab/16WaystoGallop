/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:51 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RL_hip_joint_src.h"

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
  double t4193;
  double t4179;
  double t4234;
  double t4222;
  double t4245;
  double t4247;
  double t4244;
  double t4254;
  double t4246;
  double t4248;
  double t4252;
  double t4255;
  double t4256;
  double t4257;
  double t4260;
  double t4261;
  double t4262;
  double t4264;
  double t4265;
  double t4266;
  t4193 = Cos(var1[4]);
  t4179 = Cos(var1[3]);
  t4234 = Sin(var1[4]);
  t4222 = Sin(var1[3]);
  t4245 = Cos(var1[5]);
  t4247 = Sin(var1[5]);
  t4244 = Sin(var1[15]);
  t4254 = Cos(var1[15]);
  t4246 = t4179*t4245*t4234;
  t4248 = t4222*t4247;
  t4252 = t4246 + t4248;
  t4255 = -1.*t4245*t4222;
  t4256 = t4179*t4234*t4247;
  t4257 = t4255 + t4256;
  t4260 = t4245*t4222*t4234;
  t4261 = -1.*t4179*t4247;
  t4262 = t4260 + t4261;
  t4264 = t4179*t4245;
  t4265 = t4222*t4234*t4247;
  t4266 = t4264 + t4265;
  p_output1[0]=t4179*t4193;
  p_output1[1]=t4193*t4222;
  p_output1[2]=-1.*t4234;
  p_output1[3]=t4244*t4252 + t4254*t4257;
  p_output1[4]=t4244*t4262 + t4254*t4266;
  p_output1[5]=t4193*t4244*t4245 + t4193*t4247*t4254;
  p_output1[6]=t4252*t4254 - 1.*t4244*t4257;
  p_output1[7]=t4254*t4262 - 1.*t4244*t4266;
  p_output1[8]=-1.*t4193*t4244*t4247 + t4193*t4245*t4254;
}



void R_RL_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
