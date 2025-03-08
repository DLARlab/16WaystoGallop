/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:39 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RR_hip_joint_src.h"

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
  double t2867;
  double t2855;
  double t2904;
  double t2893;
  double t2915;
  double t2917;
  double t2914;
  double t2924;
  double t2916;
  double t2921;
  double t2922;
  double t2925;
  double t2926;
  double t2927;
  double t2930;
  double t2931;
  double t2932;
  double t2934;
  double t2935;
  double t2936;
  t2867 = Cos(var1[4]);
  t2855 = Cos(var1[3]);
  t2904 = Sin(var1[4]);
  t2893 = Sin(var1[3]);
  t2915 = Cos(var1[5]);
  t2917 = Sin(var1[5]);
  t2914 = Sin(var1[12]);
  t2924 = Cos(var1[12]);
  t2916 = t2855*t2915*t2904;
  t2921 = t2893*t2917;
  t2922 = t2916 + t2921;
  t2925 = -1.*t2915*t2893;
  t2926 = t2855*t2904*t2917;
  t2927 = t2925 + t2926;
  t2930 = t2915*t2893*t2904;
  t2931 = -1.*t2855*t2917;
  t2932 = t2930 + t2931;
  t2934 = t2855*t2915;
  t2935 = t2893*t2904*t2917;
  t2936 = t2934 + t2935;
  p_output1[0]=t2855*t2867;
  p_output1[1]=t2867*t2893;
  p_output1[2]=-1.*t2904;
  p_output1[3]=t2914*t2922 + t2924*t2927;
  p_output1[4]=t2914*t2932 + t2924*t2936;
  p_output1[5]=t2867*t2914*t2915 + t2867*t2917*t2924;
  p_output1[6]=t2922*t2924 - 1.*t2914*t2927;
  p_output1[7]=t2924*t2932 - 1.*t2914*t2936;
  p_output1[8]=-1.*t2867*t2914*t2917 + t2867*t2915*t2924;
}



void R_RR_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
