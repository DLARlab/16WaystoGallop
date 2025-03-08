/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:20 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FR_thigh_joint_src.h"

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
  double t523;
  double t625;
  double t640;
  double t629;
  double t641;
  double t604;
  double t605;
  double t624;
  double t648;
  double t651;
  double t645;
  double t646;
  double t647;
  double t639;
  double t642;
  double t643;
  double t659;
  double t660;
  double t661;
  double t655;
  double t656;
  double t657;
  double t644;
  double t649;
  double t650;
  double t658;
  double t662;
  double t663;
  double t667;
  double t668;
  double t669;
  t523 = Cos(var1[3]);
  t625 = Cos(var1[5]);
  t640 = Sin(var1[3]);
  t629 = Sin(var1[4]);
  t641 = Sin(var1[5]);
  t604 = Cos(var1[4]);
  t605 = Cos(var1[7]);
  t624 = Cos(var1[6]);
  t648 = Sin(var1[6]);
  t651 = Sin(var1[7]);
  t645 = -1.*t625*t640;
  t646 = t523*t629*t641;
  t647 = t645 + t646;
  t639 = t523*t625*t629;
  t642 = t640*t641;
  t643 = t639 + t642;
  t659 = t523*t625;
  t660 = t640*t629*t641;
  t661 = t659 + t660;
  t655 = t625*t640*t629;
  t656 = -1.*t523*t641;
  t657 = t655 + t656;
  t644 = t624*t643;
  t649 = -1.*t647*t648;
  t650 = t644 + t649;
  t658 = t624*t657;
  t662 = -1.*t661*t648;
  t663 = t658 + t662;
  t667 = t604*t625*t624;
  t668 = -1.*t604*t641*t648;
  t669 = t667 + t668;
  p_output1[0]=t523*t604*t605 - 1.*t650*t651;
  p_output1[1]=t604*t605*t640 - 1.*t651*t663;
  p_output1[2]=-1.*t605*t629 - 1.*t651*t669;
  p_output1[3]=t624*t647 + t643*t648;
  p_output1[4]=t648*t657 + t624*t661;
  p_output1[5]=t604*t624*t641 + t604*t625*t648;
  p_output1[6]=t605*t650 + t523*t604*t651;
  p_output1[7]=t604*t640*t651 + t605*t663;
  p_output1[8]=-1.*t629*t651 + t605*t669;
}



void R_FR_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
