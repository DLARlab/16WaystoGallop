/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:15 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FR_hip_joint_src.h"

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
  double t157;
  double t166;
  double t153;
  double t170;
  double t178;
  double t180;
  double t176;
  double t192;
  double t189;
  double t190;
  double t191;
  double t179;
  double t182;
  double t184;
  double t203;
  double t204;
  double t205;
  double t198;
  double t199;
  double t201;
  double t188;
  double t193;
  double t194;
  double t231;
  double t233;
  double t234;
  double t236;
  double t238;
  double t239;
  double t202;
  double t207;
  double t212;
  double t213;
  double t214;
  double t215;
  t157 = Cos(var1[4]);
  t166 = Sin(var1[3]);
  t153 = Cos(var1[3]);
  t170 = Sin(var1[4]);
  t178 = Cos(var1[5]);
  t180 = Sin(var1[5]);
  t176 = Cos(var1[6]);
  t192 = Sin(var1[6]);
  t189 = t153*t178*t170;
  t190 = t166*t180;
  t191 = t189 + t190;
  t179 = -1.*t178*t166;
  t182 = t153*t170*t180;
  t184 = t179 + t182;
  t203 = t178*t166*t170;
  t204 = -1.*t153*t180;
  t205 = t203 + t204;
  t198 = t153*t178;
  t199 = t166*t170*t180;
  t201 = t198 + t199;
  t188 = t176*t184;
  t193 = t191*t192;
  t194 = t188 + t193;
  t231 = -1.*t176;
  t233 = 1. + t231;
  t234 = -0.047*t233;
  t236 = 0. + t234;
  t238 = 0.047*t192;
  t239 = 0. + t238;
  t202 = t176*t201;
  t207 = t205*t192;
  t212 = t202 + t207;
  t213 = t157*t176*t180;
  t214 = t157*t178*t192;
  t215 = t213 + t214;
  p_output1[0]=t153*t157;
  p_output1[1]=t157*t166;
  p_output1[2]=-1.*t170;
  p_output1[3]=0.;
  p_output1[4]=t194;
  p_output1[5]=t212;
  p_output1[6]=t215;
  p_output1[7]=0.;
  p_output1[8]=t176*t191 - 1.*t184*t192;
  p_output1[9]=-1.*t192*t201 + t176*t205;
  p_output1[10]=t157*t176*t178 - 1.*t157*t180*t192;
  p_output1[11]=0.;
  p_output1[12]=0. + 0.183*t153*t157 - 0.047*t194 + t184*t236 + t191*t239 + var1[0];
  p_output1[13]=0. + 0.183*t157*t166 - 0.047*t212 + t201*t236 + t205*t239 + var1[1];
  p_output1[14]=0. - 0.183*t170 - 0.047*t215 + t157*t180*t236 + t157*t178*t239 + var1[2];
  p_output1[15]=1.;
}



void H_FR_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
