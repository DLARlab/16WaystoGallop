/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:16 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FR_hip_joint_src.h"

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
  double t207;
  double t229;
  double t190;
  double t240;
  double t253;
  double t255;
  double t250;
  double t263;
  double t260;
  double t261;
  double t262;
  double t254;
  double t257;
  double t258;
  double t270;
  double t271;
  double t272;
  double t266;
  double t267;
  double t268;
  t207 = Cos(var1[4]);
  t229 = Sin(var1[3]);
  t190 = Cos(var1[3]);
  t240 = Sin(var1[4]);
  t253 = Cos(var1[5]);
  t255 = Sin(var1[5]);
  t250 = Cos(var1[6]);
  t263 = Sin(var1[6]);
  t260 = t190*t253*t240;
  t261 = t229*t255;
  t262 = t260 + t261;
  t254 = -1.*t253*t229;
  t257 = t190*t240*t255;
  t258 = t254 + t257;
  t270 = t253*t229*t240;
  t271 = -1.*t190*t255;
  t272 = t270 + t271;
  t266 = t190*t253;
  t267 = t229*t240*t255;
  t268 = t266 + t267;
  p_output1[0]=t190*t207;
  p_output1[1]=t207*t229;
  p_output1[2]=-1.*t240;
  p_output1[3]=t250*t258 + t262*t263;
  p_output1[4]=t250*t268 + t263*t272;
  p_output1[5]=t207*t250*t255 + t207*t253*t263;
  p_output1[6]=t250*t262 - 1.*t258*t263;
  p_output1[7]=-1.*t263*t268 + t250*t272;
  p_output1[8]=t207*t250*t253 - 1.*t207*t255*t263;
}



void R_FR_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
