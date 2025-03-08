/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:38 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RLfoot.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE static inline        /* use standard inline */
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
static void output1(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  double t2050;
  double t3365;
  double t4221;
  double t4131;
  double t4224;
  double t2870;
  double t4207;
  double t4229;
  double t4352;
  double t4467;
  double t5738;
  double t6018;
  double t6059;
  double t2414;
  double t6246;
  double t35;
  double t6450;
  double t6455;
  double t6482;
  double t6486;
  double t6489;
  double t6497;
  double t6502;
  double t6543;
  double t6546;
  double t6547;
  double t6553;
  double t6555;
  double t6556;
  double t298;
  double t304;
  double t1391;
  double t1639;
  double t3043;
  double t3057;
  double t4928;
  double t4929;
  double t4938;
  double t5715;
  double t6607;
  double t6613;
  double t6614;
  double t6621;
  double t6622;
  double t6627;
  double t6281;
  double t6357;
  double t6501;
  double t6519;
  double t6533;
  double t6639;
  double t6641;
  double t6642;
  double t6549;
  double t6550;
  double t6551;
  double t6658;
  double t6674;
  double t6713;
  double t6777;
  double t6818;
  double t6820;
  double t7349;
  double t7366;
  double t7379;
  double t7416;
  double t7421;
  double t7428;
  double t7441;
  double t7459;
  double t7499;
  t2050 = Cos(var1[3]);
  t3365 = Cos(var1[5]);
  t4221 = Sin(var1[3]);
  t4131 = Sin(var1[4]);
  t4224 = Sin(var1[5]);
  t2870 = Sin(var1[15]);
  t4207 = t2050*t3365*t4131;
  t4229 = t4221*t4224;
  t4352 = t4207 + t4229;
  t4467 = Cos(var1[15]);
  t5738 = -1.*t3365*t4221;
  t6018 = t2050*t4131*t4224;
  t6059 = t5738 + t6018;
  t2414 = Cos(var1[4]);
  t6246 = Sin(var1[16]);
  t35 = Cos(var1[16]);
  t6450 = t4467*t4352;
  t6455 = -1.*t2870*t6059;
  t6482 = t6450 + t6455;
  t6486 = Cos(var1[17]);
  t6489 = -1.*t6486;
  t6497 = 1. + t6489;
  t6502 = Sin(var1[17]);
  t6543 = t2050*t2414*t6246;
  t6546 = t35*t6482;
  t6547 = t6543 + t6546;
  t6553 = t35*t2050*t2414;
  t6555 = -1.*t6246*t6482;
  t6556 = t6553 + t6555;
  t298 = -1.*t35;
  t304 = 1. + t298;
  t1391 = -0.183*t304;
  t1639 = 0. + t1391;
  t3043 = -0.047*t2870;
  t3057 = 0. + t3043;
  t4928 = -1.*t4467;
  t4929 = 1. + t4928;
  t4938 = 0.047*t4929;
  t5715 = 0. + t4938;
  t6607 = t3365*t4221*t4131;
  t6613 = -1.*t2050*t4224;
  t6614 = t6607 + t6613;
  t6621 = t2050*t3365;
  t6622 = t4221*t4131*t4224;
  t6627 = t6621 + t6622;
  t6281 = -0.183*t6246;
  t6357 = 0. + t6281;
  t6501 = -0.2*t6497;
  t6519 = -0.183*t6502;
  t6533 = 0. + t6501 + t6519;
  t6639 = t4467*t6614;
  t6641 = -1.*t2870*t6627;
  t6642 = t6639 + t6641;
  t6549 = -0.183*t6497;
  t6550 = 0.2*t6502;
  t6551 = 0. + t6549 + t6550;
  t6658 = t2414*t6246*t4221;
  t6674 = t35*t6642;
  t6713 = t6658 + t6674;
  t6777 = t35*t2414*t4221;
  t6818 = -1.*t6246*t6642;
  t6820 = t6777 + t6818;
  t7349 = t4467*t2414*t3365;
  t7366 = -1.*t2414*t2870*t4224;
  t7379 = t7349 + t7366;
  t7416 = -1.*t6246*t4131;
  t7421 = t35*t7379;
  t7428 = t7416 + t7421;
  t7441 = -1.*t35*t4131;
  t7459 = -1.*t6246*t7379;
  t7499 = t7441 + t7459;

  p_output1(0)=0. + t1639*t2050*t2414 + t3057*t4352 + t5715*t6059 + 0.13205*(t2870*t4352 + t4467*t6059) + t6357*t6482 + t6533*t6547 + t6551*t6556 - 0.183*(-1.*t6502*t6547 + t6486*t6556) - 0.4*(t6486*t6547 + t6502*t6556) + var1(0);
  p_output1(1)=0. + t1639*t2414*t4221 + t3057*t6614 + t5715*t6627 + 0.13205*(t2870*t6614 + t4467*t6627) + t6357*t6642 + t6533*t6713 + t6551*t6820 - 0.183*(-1.*t6502*t6713 + t6486*t6820) - 0.4*(t6486*t6713 + t6502*t6820) + var1(1);
  p_output1(2)=0. + t2414*t3057*t3365 - 1.*t1639*t4131 + 0.13205*(t2414*t2870*t3365 + t2414*t4224*t4467) + t2414*t4224*t5715 + t6357*t7379 + t6533*t7428 + t6551*t7499 - 0.183*(-1.*t6502*t7428 + t6486*t7499) - 0.4*(t6486*t7428 + t6502*t7499) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_RLfoot(const Eigen::Matrix<double,18,1> &var1)
//void p_RLfoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



