/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:55 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_RL_thigh_joint_src.h"

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
  double t4497;
  double t4613;
  double t4624;
  double t4614;
  double t4625;
  double t174;
  double t4592;
  double t4594;
  double t4611;
  double t4633;
  double t4622;
  double t4630;
  double t4631;
  double t4634;
  double t4635;
  double t4636;
  double t4642;
  double t4643;
  double t4644;
  double t4646;
  double t4647;
  double t4648;
  double t4632;
  double t4637;
  double t4638;
  double t4645;
  double t4649;
  double t4650;
  double t4654;
  double t4655;
  double t4656;
  t4497 = Cos(var1[3]);
  t4613 = Cos(var1[5]);
  t4624 = Sin(var1[3]);
  t4614 = Sin(var1[4]);
  t4625 = Sin(var1[5]);
  t174 = Cos(var1[16]);
  t4592 = Cos(var1[4]);
  t4594 = Sin(var1[16]);
  t4611 = Cos(var1[15]);
  t4633 = Sin(var1[15]);
  t4622 = t4497*t4613*t4614;
  t4630 = t4624*t4625;
  t4631 = t4622 + t4630;
  t4634 = -1.*t4613*t4624;
  t4635 = t4497*t4614*t4625;
  t4636 = t4634 + t4635;
  t4642 = t4613*t4624*t4614;
  t4643 = -1.*t4497*t4625;
  t4644 = t4642 + t4643;
  t4646 = t4497*t4613;
  t4647 = t4624*t4614*t4625;
  t4648 = t4646 + t4647;
  t4632 = t4611*t4631;
  t4637 = -1.*t4633*t4636;
  t4638 = t4632 + t4637;
  t4645 = t4611*t4644;
  t4649 = -1.*t4633*t4648;
  t4650 = t4645 + t4649;
  t4654 = t4611*t4592*t4613;
  t4655 = -1.*t4592*t4633*t4625;
  t4656 = t4654 + t4655;
  p_output1[0]=t174*t4497*t4592 - 1.*t4594*t4638;
  p_output1[1]=t174*t4592*t4624 - 1.*t4594*t4650;
  p_output1[2]=-1.*t174*t4614 - 1.*t4594*t4656;
  p_output1[3]=t4631*t4633 + t4611*t4636;
  p_output1[4]=t4633*t4644 + t4611*t4648;
  p_output1[5]=t4592*t4611*t4625 + t4592*t4613*t4633;
  p_output1[6]=t4497*t4592*t4594 + t174*t4638;
  p_output1[7]=t4592*t4594*t4624 + t174*t4650;
  p_output1[8]=-1.*t4594*t4614 + t174*t4656;
}



void R_RL_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
