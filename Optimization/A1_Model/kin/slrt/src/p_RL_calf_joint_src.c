/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:56 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RL_calf_joint_src.h"

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
  double t4694;
  double t4706;
  double t4710;
  double t4708;
  double t4711;
  double t4699;
  double t4709;
  double t4712;
  double t4714;
  double t4716;
  double t4727;
  double t4728;
  double t4730;
  double t4697;
  double t4738;
  double t205;
  double t4741;
  double t4742;
  double t4743;
  double t4745;
  double t4746;
  double t4747;
  double t4750;
  double t4753;
  double t4755;
  double t4756;
  double t4763;
  double t4764;
  double t4765;
  double t4672;
  double t4688;
  double t4690;
  double t4691;
  double t4703;
  double t4704;
  double t4719;
  double t4723;
  double t4725;
  double t4726;
  double t4780;
  double t4781;
  double t4782;
  double t4784;
  double t4785;
  double t4786;
  double t4739;
  double t4740;
  double t4748;
  double t4751;
  double t4752;
  double t4799;
  double t4800;
  double t4801;
  double t4758;
  double t4759;
  double t4760;
  double t4803;
  double t4804;
  double t4805;
  double t4807;
  double t4808;
  double t4809;
  double t4827;
  double t4828;
  double t4829;
  double t4831;
  double t4832;
  double t4833;
  double t4835;
  double t4836;
  double t4837;
  t4694 = Cos(var1[3]);
  t4706 = Cos(var1[5]);
  t4710 = Sin(var1[3]);
  t4708 = Sin(var1[4]);
  t4711 = Sin(var1[5]);
  t4699 = Sin(var1[15]);
  t4709 = t4694*t4706*t4708;
  t4712 = t4710*t4711;
  t4714 = t4709 + t4712;
  t4716 = Cos(var1[15]);
  t4727 = -1.*t4706*t4710;
  t4728 = t4694*t4708*t4711;
  t4730 = t4727 + t4728;
  t4697 = Cos(var1[4]);
  t4738 = Sin(var1[16]);
  t205 = Cos(var1[16]);
  t4741 = t4716*t4714;
  t4742 = -1.*t4699*t4730;
  t4743 = t4741 + t4742;
  t4745 = Cos(var1[17]);
  t4746 = -1.*t4745;
  t4747 = 1. + t4746;
  t4750 = Sin(var1[17]);
  t4753 = t4694*t4697*t4738;
  t4755 = t205*t4743;
  t4756 = t4753 + t4755;
  t4763 = t205*t4694*t4697;
  t4764 = -1.*t4738*t4743;
  t4765 = t4763 + t4764;
  t4672 = -1.*t205;
  t4688 = 1. + t4672;
  t4690 = -0.183*t4688;
  t4691 = 0. + t4690;
  t4703 = -0.047*t4699;
  t4704 = 0. + t4703;
  t4719 = -1.*t4716;
  t4723 = 1. + t4719;
  t4725 = 0.047*t4723;
  t4726 = 0. + t4725;
  t4780 = t4706*t4710*t4708;
  t4781 = -1.*t4694*t4711;
  t4782 = t4780 + t4781;
  t4784 = t4694*t4706;
  t4785 = t4710*t4708*t4711;
  t4786 = t4784 + t4785;
  t4739 = -0.183*t4738;
  t4740 = 0. + t4739;
  t4748 = -0.2*t4747;
  t4751 = -0.183*t4750;
  t4752 = 0. + t4748 + t4751;
  t4799 = t4716*t4782;
  t4800 = -1.*t4699*t4786;
  t4801 = t4799 + t4800;
  t4758 = -0.183*t4747;
  t4759 = 0.2*t4750;
  t4760 = 0. + t4758 + t4759;
  t4803 = t4697*t4738*t4710;
  t4804 = t205*t4801;
  t4805 = t4803 + t4804;
  t4807 = t205*t4697*t4710;
  t4808 = -1.*t4738*t4801;
  t4809 = t4807 + t4808;
  t4827 = t4716*t4697*t4706;
  t4828 = -1.*t4697*t4699*t4711;
  t4829 = t4827 + t4828;
  t4831 = -1.*t4738*t4708;
  t4832 = t205*t4829;
  t4833 = t4831 + t4832;
  t4835 = -1.*t205*t4708;
  t4836 = -1.*t4738*t4829;
  t4837 = t4835 + t4836;
  p_output1[0]=0. + t4691*t4694*t4697 + t4704*t4714 + t4726*t4730 + 0.13205*(t4699*t4714 + t4716*t4730) + t4740*t4743 + t4752*t4756 + t4760*t4765 - 0.183*(-1.*t4750*t4756 + t4745*t4765) - 0.2*(t4745*t4756 + t4750*t4765) + var1[0];
  p_output1[1]=0. + t4691*t4697*t4710 + t4704*t4782 + t4726*t4786 + 0.13205*(t4699*t4782 + t4716*t4786) + t4740*t4801 + t4752*t4805 + t4760*t4809 - 0.183*(-1.*t4750*t4805 + t4745*t4809) - 0.2*(t4745*t4805 + t4750*t4809) + var1[1];
  p_output1[2]=0. + t4697*t4704*t4706 - 1.*t4691*t4708 + 0.13205*(t4697*t4699*t4706 + t4697*t4711*t4716) + t4697*t4711*t4726 + t4740*t4829 + t4752*t4833 + t4760*t4837 - 0.183*(-1.*t4750*t4833 + t4745*t4837) - 0.2*(t4745*t4833 + t4750*t4837) + var1[2];
}



void p_RL_calf_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
