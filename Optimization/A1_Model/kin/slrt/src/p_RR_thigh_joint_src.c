/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:40 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RR_thigh_joint_src.h"

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
  double t2966;
  double t2977;
  double t2980;
  double t2978;
  double t2981;
  double t2972;
  double t2979;
  double t2983;
  double t2984;
  double t2988;
  double t2993;
  double t2994;
  double t2995;
  double t2943;
  double t2967;
  double t3003;
  double t3006;
  double t3007;
  double t3010;
  double t2957;
  double t2959;
  double t2960;
  double t2963;
  double t2973;
  double t2975;
  double t2989;
  double t2990;
  double t2991;
  double t2992;
  double t3018;
  double t3019;
  double t3020;
  double t3022;
  double t3023;
  double t3026;
  double t3004;
  double t3005;
  double t3032;
  double t3033;
  double t3034;
  double t3048;
  double t3049;
  double t3050;
  t2966 = Cos(var1[3]);
  t2977 = Cos(var1[5]);
  t2980 = Sin(var1[3]);
  t2978 = Sin(var1[4]);
  t2981 = Sin(var1[5]);
  t2972 = Sin(var1[12]);
  t2979 = t2966*t2977*t2978;
  t2983 = t2980*t2981;
  t2984 = t2979 + t2983;
  t2988 = Cos(var1[12]);
  t2993 = -1.*t2977*t2980;
  t2994 = t2966*t2978*t2981;
  t2995 = t2993 + t2994;
  t2943 = Cos(var1[13]);
  t2967 = Cos(var1[4]);
  t3003 = Sin(var1[13]);
  t3006 = t2988*t2984;
  t3007 = -1.*t2972*t2995;
  t3010 = t3006 + t3007;
  t2957 = -1.*t2943;
  t2959 = 1. + t2957;
  t2960 = -0.183*t2959;
  t2963 = 0. + t2960;
  t2973 = 0.047*t2972;
  t2975 = 0. + t2973;
  t2989 = -1.*t2988;
  t2990 = 1. + t2989;
  t2991 = -0.047*t2990;
  t2992 = 0. + t2991;
  t3018 = t2977*t2980*t2978;
  t3019 = -1.*t2966*t2981;
  t3020 = t3018 + t3019;
  t3022 = t2966*t2977;
  t3023 = t2980*t2978*t2981;
  t3026 = t3022 + t3023;
  t3004 = -0.183*t3003;
  t3005 = 0. + t3004;
  t3032 = t2988*t3020;
  t3033 = -1.*t2972*t3026;
  t3034 = t3032 + t3033;
  t3048 = t2988*t2967*t2977;
  t3049 = -1.*t2967*t2972*t2981;
  t3050 = t3048 + t3049;
  p_output1[0]=0. + t2963*t2966*t2967 + t2975*t2984 + t2992*t2995 - 0.13205*(t2972*t2984 + t2988*t2995) + t3005*t3010 - 0.183*(t2943*t2966*t2967 - 1.*t3003*t3010) + var1[0];
  p_output1[1]=0. + t2963*t2967*t2980 + t2975*t3020 + t2992*t3026 - 0.13205*(t2972*t3020 + t2988*t3026) + t3005*t3034 - 0.183*(t2943*t2967*t2980 - 1.*t3003*t3034) + var1[1];
  p_output1[2]=0. + t2967*t2975*t2977 - 1.*t2963*t2978 - 0.13205*(t2967*t2972*t2977 + t2967*t2981*t2988) + t2967*t2981*t2992 + t3005*t3050 - 0.183*(-1.*t2943*t2978 - 1.*t3003*t3050) + var1[2];
}



void p_RR_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
