/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:36 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RR_hip_joint_src.h"

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
  double t2717;
  double t2751;
  double t2756;
  double t2754;
  double t2760;
  double t2738;
  double t2755;
  double t2761;
  double t2763;
  double t2765;
  double t2770;
  double t2771;
  double t2772;
  double t2718;
  double t2739;
  double t2750;
  double t2766;
  double t2767;
  double t2768;
  double t2769;
  double t2781;
  double t2782;
  double t2783;
  double t2785;
  double t2786;
  double t2787;
  t2717 = Cos(var1[3]);
  t2751 = Cos(var1[5]);
  t2756 = Sin(var1[3]);
  t2754 = Sin(var1[4]);
  t2760 = Sin(var1[5]);
  t2738 = Sin(var1[12]);
  t2755 = t2717*t2751*t2754;
  t2761 = t2756*t2760;
  t2763 = t2755 + t2761;
  t2765 = Cos(var1[12]);
  t2770 = -1.*t2751*t2756;
  t2771 = t2717*t2754*t2760;
  t2772 = t2770 + t2771;
  t2718 = Cos(var1[4]);
  t2739 = 0.047*t2738;
  t2750 = 0. + t2739;
  t2766 = -1.*t2765;
  t2767 = 1. + t2766;
  t2768 = -0.047*t2767;
  t2769 = 0. + t2768;
  t2781 = t2751*t2756*t2754;
  t2782 = -1.*t2717*t2760;
  t2783 = t2781 + t2782;
  t2785 = t2717*t2751;
  t2786 = t2756*t2754*t2760;
  t2787 = t2785 + t2786;
  p_output1[0]=0. - 0.183*t2717*t2718 + t2750*t2763 + t2769*t2772 - 0.047*(t2738*t2763 + t2765*t2772) + var1[0];
  p_output1[1]=0. - 0.183*t2718*t2756 + t2750*t2783 + t2769*t2787 - 0.047*(t2738*t2783 + t2765*t2787) + var1[1];
  p_output1[2]=0. + t2718*t2750*t2751 + 0.183*t2754 - 0.047*(t2718*t2738*t2751 + t2718*t2760*t2765) + t2718*t2760*t2769 + var1[2];
}



void p_RR_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
