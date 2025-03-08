/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:11 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_BaseRotX_src.h"

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
  double t47;
  double t51;
  double t45;
  double t54;
  double t56;
  double t58;
  t47 = Cos(var1[4]);
  t51 = Sin(var1[3]);
  t45 = Cos(var1[3]);
  t54 = Sin(var1[4]);
  t56 = Cos(var1[5]);
  t58 = Sin(var1[5]);
  p_output1[0]=t45*t47;
  p_output1[1]=t47*t51;
  p_output1[2]=-1.*t54;
  p_output1[3]=0;
  p_output1[4]=-1.*t51*t56 + t45*t54*t58;
  p_output1[5]=t45*t56 + t51*t54*t58;
  p_output1[6]=t47*t58;
  p_output1[7]=0;
  p_output1[8]=t45*t54*t56 + t51*t58;
  p_output1[9]=t51*t54*t56 - 1.*t45*t58;
  p_output1[10]=t47*t56;
  p_output1[11]=0;
  p_output1[12]=var1[0];
  p_output1[13]=var1[1];
  p_output1[14]=var1[2];
  p_output1[15]=1.;
}



void H_BaseRotX_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
