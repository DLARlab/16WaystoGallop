/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:10 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RRfoot_src.h"

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
  double t1266;
  double t2527;
  double t6893;
  double t5054;
  double t6896;
  double t1713;
  double t1267;
  double t1272;
  double t1715;
  double t6819;
  double t6950;
  double t6959;
  double t6968;
  double t6989;
  double t7008;
  double t7013;
  double t7015;
  double t7017;
  double t7021;
  double t1261;
  double t7038;
  double t7045;
  double t7047;
  double t7056;
  double t7065;
  double t7070;
  double t7071;
  double t7074;
  double t7078;
  double t7079;
  double t7094;
  double t7096;
  double t7100;
  double t1276;
  double t7025;
  double t7030;
  double t7039;
  double t7040;
  double t7041;
  double t7044;
  double t7081;
  double t7082;
  double t7084;
  double t7085;
  double t7086;
  double t7093;
  double t7102;
  double t7107;
  double t7111;
  double t7112;
  double t7113;
  t1266 = Cos(var1[3]);
  t2527 = Cos(var1[5]);
  t6893 = Sin(var1[3]);
  t5054 = Sin(var1[4]);
  t6896 = Sin(var1[5]);
  t1713 = Cos(var1[13]);
  t1267 = Cos(var1[4]);
  t1272 = Sin(var1[13]);
  t1715 = Cos(var1[12]);
  t6819 = t1266*t2527*t5054;
  t6950 = t6893*t6896;
  t6959 = t6819 + t6950;
  t6968 = t1715*t6959;
  t6989 = Sin(var1[12]);
  t7008 = -1.*t2527*t6893;
  t7013 = t1266*t5054*t6896;
  t7015 = t7008 + t7013;
  t7017 = -1.*t6989*t7015;
  t7021 = t6968 + t7017;
  t1261 = Sin(var1[14]);
  t7038 = Cos(var1[14]);
  t7045 = t2527*t6893*t5054;
  t7047 = -1.*t1266*t6896;
  t7056 = t7045 + t7047;
  t7065 = t1715*t7056;
  t7070 = t1266*t2527;
  t7071 = t6893*t5054*t6896;
  t7074 = t7070 + t7071;
  t7078 = -1.*t6989*t7074;
  t7079 = t7065 + t7078;
  t7094 = t1715*t1267*t2527;
  t7096 = -1.*t1267*t6989*t6896;
  t7100 = t7094 + t7096;
  t1276 = t1266*t1267*t1272;
  t7025 = t1713*t7021;
  t7030 = t1276 + t7025;
  t7039 = t1713*t1266*t1267;
  t7040 = -1.*t1272*t7021;
  t7041 = t7039 + t7040;
  t7044 = t1267*t1272*t6893;
  t7081 = t1713*t7079;
  t7082 = t7044 + t7081;
  t7084 = t1713*t1267*t6893;
  t7085 = -1.*t1272*t7079;
  t7086 = t7084 + t7085;
  t7093 = -1.*t1272*t5054;
  t7102 = t1713*t7100;
  t7107 = t7093 + t7102;
  t7111 = -1.*t1713*t5054;
  t7112 = -1.*t1272*t7100;
  t7113 = t7111 + t7112;
  p_output1[0]=-1.*t1261*t7030 + t7038*t7041;
  p_output1[1]=-1.*t1261*t7082 + t7038*t7086;
  p_output1[2]=-1.*t1261*t7107 + t7038*t7113;
  p_output1[3]=t6959*t6989 + t1715*t7015;
  p_output1[4]=t6989*t7056 + t1715*t7074;
  p_output1[5]=t1267*t1715*t6896 + t1267*t2527*t6989;
  p_output1[6]=t7030*t7038 + t1261*t7041;
  p_output1[7]=t7038*t7082 + t1261*t7086;
  p_output1[8]=t7038*t7107 + t1261*t7113;
}



void R_RRfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
