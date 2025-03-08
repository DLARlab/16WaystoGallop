/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:17 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FR_thigh_joint_src.h"

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
  double t285;
  double t315;
  double t318;
  double t316;
  double t322;
  double t309;
  double t317;
  double t323;
  double t324;
  double t326;
  double t327;
  double t328;
  double t329;
  double t293;
  double t297;
  double t339;
  double t340;
  double t341;
  double t342;
  double t298;
  double t304;
  double t306;
  double t307;
  double t311;
  double t312;
  double t313;
  double t314;
  double t330;
  double t331;
  double t354;
  double t355;
  double t356;
  double t358;
  double t359;
  double t362;
  double t343;
  double t346;
  double t368;
  double t369;
  double t370;
  double t384;
  double t385;
  double t386;
  t285 = Cos(var1[3]);
  t315 = Cos(var1[5]);
  t318 = Sin(var1[4]);
  t316 = Sin(var1[3]);
  t322 = Sin(var1[5]);
  t309 = Cos(var1[6]);
  t317 = -1.*t315*t316;
  t323 = t285*t318*t322;
  t324 = t317 + t323;
  t326 = t285*t315*t318;
  t327 = t316*t322;
  t328 = t326 + t327;
  t329 = Sin(var1[6]);
  t293 = Cos(var1[4]);
  t297 = Cos(var1[7]);
  t339 = t309*t328;
  t340 = -1.*t324*t329;
  t341 = t339 + t340;
  t342 = Sin(var1[7]);
  t298 = -1.*t297;
  t304 = 1. + t298;
  t306 = 0.183*t304;
  t307 = 0. + t306;
  t311 = -1.*t309;
  t312 = 1. + t311;
  t313 = -0.047*t312;
  t314 = 0. + t313;
  t330 = 0.047*t329;
  t331 = 0. + t330;
  t354 = t285*t315;
  t355 = t316*t318*t322;
  t356 = t354 + t355;
  t358 = t315*t316*t318;
  t359 = -1.*t285*t322;
  t362 = t358 + t359;
  t343 = 0.183*t342;
  t346 = 0. + t343;
  t368 = t309*t362;
  t369 = -1.*t356*t329;
  t370 = t368 + t369;
  t384 = t293*t315*t309;
  t385 = -1.*t293*t322*t329;
  t386 = t384 + t385;
  p_output1[0]=0. + t285*t293*t307 + t314*t324 - 0.13205*(t309*t324 + t328*t329) + t328*t331 + 0.183*(t285*t293*t297 - 1.*t341*t342) + t341*t346 + var1[0];
  p_output1[1]=0. + t293*t307*t316 + t314*t356 + t331*t362 - 0.13205*(t309*t356 + t329*t362) + t346*t370 + 0.183*(t293*t297*t316 - 1.*t342*t370) + var1[1];
  p_output1[2]=0. - 1.*t307*t318 + t293*t314*t322 - 0.13205*(t293*t309*t322 + t293*t315*t329) + t293*t315*t331 + t346*t386 + 0.183*(-1.*t297*t318 - 1.*t342*t386) + var1[2];
}



void p_FR_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
