/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:13 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FR_hip_joint_src.h"

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
  double t77;
  double t95;
  double t98;
  double t96;
  double t99;
  double t86;
  double t97;
  double t100;
  double t101;
  double t103;
  double t104;
  double t105;
  double t106;
  double t79;
  double t90;
  double t92;
  double t93;
  double t94;
  double t107;
  double t108;
  double t117;
  double t118;
  double t119;
  double t121;
  double t122;
  double t123;
  t77 = Cos(var1[3]);
  t95 = Cos(var1[5]);
  t98 = Sin(var1[4]);
  t96 = Sin(var1[3]);
  t99 = Sin(var1[5]);
  t86 = Cos(var1[6]);
  t97 = -1.*t95*t96;
  t100 = t77*t98*t99;
  t101 = t97 + t100;
  t103 = t77*t95*t98;
  t104 = t96*t99;
  t105 = t103 + t104;
  t106 = Sin(var1[6]);
  t79 = Cos(var1[4]);
  t90 = -1.*t86;
  t92 = 1. + t90;
  t93 = -0.047*t92;
  t94 = 0. + t93;
  t107 = 0.047*t106;
  t108 = 0. + t107;
  t117 = t77*t95;
  t118 = t96*t98*t99;
  t119 = t117 + t118;
  t121 = t95*t96*t98;
  t122 = -1.*t77*t99;
  t123 = t121 + t122;
  p_output1[0]=0. + t105*t108 + 0.183*t77*t79 - 0.047*(t105*t106 + t101*t86) + t101*t94 + var1[0];
  p_output1[1]=0. + t108*t123 - 0.047*(t106*t123 + t119*t86) + t119*t94 + 0.183*t79*t96 + var1[1];
  p_output1[2]=0. + t108*t79*t95 - 0.183*t98 + t79*t94*t99 - 0.047*(t106*t79*t95 + t79*t86*t99) + var1[2];
}



void p_FR_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
