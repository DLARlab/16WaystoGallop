/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:14 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RLfoot_src.h"

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
  double t2195;
  double t3443;
  double t4371;
  double t3447;
  double t7409;
  double t3215;
  double t3133;
  double t3174;
  double t3277;
  double t3913;
  double t7440;
  double t7451;
  double t7505;
  double t7509;
  double t7510;
  double t7538;
  double t7561;
  double t7564;
  double t7565;
  double t2191;
  double t7573;
  double t7590;
  double t7591;
  double t7592;
  double t7593;
  double t7594;
  double t7595;
  double t7596;
  double t7597;
  double t7598;
  double t7634;
  double t7635;
  double t7639;
  double t3185;
  double t7566;
  double t7567;
  double t7575;
  double t7576;
  double t7577;
  double t7589;
  double t7599;
  double t7602;
  double t7608;
  double t7616;
  double t7625;
  double t7633;
  double t7640;
  double t7644;
  double t7647;
  double t7648;
  double t7649;
  t2195 = Cos(var1[3]);
  t3443 = Cos(var1[5]);
  t4371 = Sin(var1[3]);
  t3447 = Sin(var1[4]);
  t7409 = Sin(var1[5]);
  t3215 = Cos(var1[16]);
  t3133 = Cos(var1[4]);
  t3174 = Sin(var1[16]);
  t3277 = Cos(var1[15]);
  t3913 = t2195*t3443*t3447;
  t7440 = t4371*t7409;
  t7451 = t3913 + t7440;
  t7505 = t3277*t7451;
  t7509 = Sin(var1[15]);
  t7510 = -1.*t3443*t4371;
  t7538 = t2195*t3447*t7409;
  t7561 = t7510 + t7538;
  t7564 = -1.*t7509*t7561;
  t7565 = t7505 + t7564;
  t2191 = Sin(var1[17]);
  t7573 = Cos(var1[17]);
  t7590 = t3443*t4371*t3447;
  t7591 = -1.*t2195*t7409;
  t7592 = t7590 + t7591;
  t7593 = t3277*t7592;
  t7594 = t2195*t3443;
  t7595 = t4371*t3447*t7409;
  t7596 = t7594 + t7595;
  t7597 = -1.*t7509*t7596;
  t7598 = t7593 + t7597;
  t7634 = t3277*t3133*t3443;
  t7635 = -1.*t3133*t7509*t7409;
  t7639 = t7634 + t7635;
  t3185 = t2195*t3133*t3174;
  t7566 = t3215*t7565;
  t7567 = t3185 + t7566;
  t7575 = t3215*t2195*t3133;
  t7576 = -1.*t3174*t7565;
  t7577 = t7575 + t7576;
  t7589 = t3133*t3174*t4371;
  t7599 = t3215*t7598;
  t7602 = t7589 + t7599;
  t7608 = t3215*t3133*t4371;
  t7616 = -1.*t3174*t7598;
  t7625 = t7608 + t7616;
  t7633 = -1.*t3174*t3447;
  t7640 = t3215*t7639;
  t7644 = t7633 + t7640;
  t7647 = -1.*t3215*t3447;
  t7648 = -1.*t3174*t7639;
  t7649 = t7647 + t7648;
  p_output1[0]=-1.*t2191*t7567 + t7573*t7577;
  p_output1[1]=-1.*t2191*t7602 + t7573*t7625;
  p_output1[2]=-1.*t2191*t7644 + t7573*t7649;
  p_output1[3]=t7451*t7509 + t3277*t7561;
  p_output1[4]=t7509*t7592 + t3277*t7596;
  p_output1[5]=t3133*t3277*t7409 + t3133*t3443*t7509;
  p_output1[6]=t7567*t7573 + t2191*t7577;
  p_output1[7]=t7573*t7602 + t2191*t7625;
  p_output1[8]=t7573*t7644 + t2191*t7649;
}



void R_RLfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
