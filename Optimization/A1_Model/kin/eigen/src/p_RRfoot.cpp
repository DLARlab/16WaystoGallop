/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:03:34 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_RRfoot.h"

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
  double t2103;
  double t2395;
  double t2621;
  double t2434;
  double t4127;
  double t2382;
  double t2470;
  double t4160;
  double t4348;
  double t4828;
  double t5707;
  double t5710;
  double t5800;
  double t2133;
  double t6313;
  double t357;
  double t6391;
  double t6406;
  double t6418;
  double t6445;
  double t6479;
  double t6657;
  double t6660;
  double t6866;
  double t6897;
  double t6921;
  double t6999;
  double t7003;
  double t7031;
  double t379;
  double t508;
  double t512;
  double t513;
  double t2387;
  double t2391;
  double t5073;
  double t5323;
  double t5330;
  double t5510;
  double t7416;
  double t7419;
  double t7421;
  double t7458;
  double t7459;
  double t7508;
  double t6347;
  double t6374;
  double t6658;
  double t6750;
  double t6778;
  double t7554;
  double t7734;
  double t7791;
  double t6979;
  double t6993;
  double t6994;
  double t7939;
  double t8040;
  double t8079;
  double t8125;
  double t8162;
  double t8182;
  double t9286;
  double t9306;
  double t9335;
  double t9350;
  double t9351;
  double t9352;
  double t9354;
  double t9355;
  double t9356;
  t2103 = Cos(var1[3]);
  t2395 = Cos(var1[5]);
  t2621 = Sin(var1[3]);
  t2434 = Sin(var1[4]);
  t4127 = Sin(var1[5]);
  t2382 = Sin(var1[12]);
  t2470 = t2103*t2395*t2434;
  t4160 = t2621*t4127;
  t4348 = t2470 + t4160;
  t4828 = Cos(var1[12]);
  t5707 = -1.*t2395*t2621;
  t5710 = t2103*t2434*t4127;
  t5800 = t5707 + t5710;
  t2133 = Cos(var1[4]);
  t6313 = Sin(var1[13]);
  t357 = Cos(var1[13]);
  t6391 = t4828*t4348;
  t6406 = -1.*t2382*t5800;
  t6418 = t6391 + t6406;
  t6445 = Cos(var1[14]);
  t6479 = -1.*t6445;
  t6657 = 1. + t6479;
  t6660 = Sin(var1[14]);
  t6866 = t2103*t2133*t6313;
  t6897 = t357*t6418;
  t6921 = t6866 + t6897;
  t6999 = t357*t2103*t2133;
  t7003 = -1.*t6313*t6418;
  t7031 = t6999 + t7003;
  t379 = -1.*t357;
  t508 = 1. + t379;
  t512 = -0.183*t508;
  t513 = 0. + t512;
  t2387 = 0.047*t2382;
  t2391 = 0. + t2387;
  t5073 = -1.*t4828;
  t5323 = 1. + t5073;
  t5330 = -0.047*t5323;
  t5510 = 0. + t5330;
  t7416 = t2395*t2621*t2434;
  t7419 = -1.*t2103*t4127;
  t7421 = t7416 + t7419;
  t7458 = t2103*t2395;
  t7459 = t2621*t2434*t4127;
  t7508 = t7458 + t7459;
  t6347 = -0.183*t6313;
  t6374 = 0. + t6347;
  t6658 = -0.2*t6657;
  t6750 = -0.183*t6660;
  t6778 = 0. + t6658 + t6750;
  t7554 = t4828*t7421;
  t7734 = -1.*t2382*t7508;
  t7791 = t7554 + t7734;
  t6979 = -0.183*t6657;
  t6993 = 0.2*t6660;
  t6994 = 0. + t6979 + t6993;
  t7939 = t2133*t6313*t2621;
  t8040 = t357*t7791;
  t8079 = t7939 + t8040;
  t8125 = t357*t2133*t2621;
  t8162 = -1.*t6313*t7791;
  t8182 = t8125 + t8162;
  t9286 = t4828*t2133*t2395;
  t9306 = -1.*t2133*t2382*t4127;
  t9335 = t9286 + t9306;
  t9350 = -1.*t6313*t2434;
  t9351 = t357*t9335;
  t9352 = t9350 + t9351;
  t9354 = -1.*t357*t2434;
  t9355 = -1.*t6313*t9335;
  t9356 = t9354 + t9355;

  p_output1(0)=0. + t2391*t4348 + t2103*t2133*t513 + t5510*t5800 - 0.13205*(t2382*t4348 + t4828*t5800) + t6374*t6418 + t6778*t6921 + t6994*t7031 - 0.183*(-1.*t6660*t6921 + t6445*t7031) - 0.4*(t6445*t6921 + t6660*t7031) + var1(0);
  p_output1(1)=0. + t2133*t2621*t513 + t2391*t7421 + t5510*t7508 - 0.13205*(t2382*t7421 + t4828*t7508) + t6374*t7791 + t6778*t8079 + t6994*t8182 - 0.183*(-1.*t6660*t8079 + t6445*t8182) - 0.4*(t6445*t8079 + t6660*t8182) + var1(1);
  p_output1(2)=0. + t2133*t2391*t2395 - 0.13205*(t2133*t2382*t2395 + t2133*t4127*t4828) - 1.*t2434*t513 + t2133*t4127*t5510 + t6374*t9335 + t6778*t9352 + t6994*t9356 - 0.183*(-1.*t6660*t9352 + t6445*t9356) - 0.4*(t6445*t9352 + t6660*t9356) + var1(2);
}


       
Eigen::Matrix<double,3,1> p_RRfoot(const Eigen::Matrix<double,18,1> &var1)
//void p_RRfoot(Eigen::Matrix<double,3,1> &p_output1, const Eigen::Matrix<double,18,1> &var1)
{
  /* Call Subroutines */
  Eigen::Matrix<double,3,1>  p_output1;
  
  output1(p_output1, var1);

  return p_output1;
}



