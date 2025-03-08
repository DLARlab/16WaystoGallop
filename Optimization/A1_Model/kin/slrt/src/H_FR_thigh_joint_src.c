/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:19 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FR_thigh_joint_src.h"

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
  double t432;
  double t458;
  double t473;
  double t465;
  double t475;
  double t444;
  double t447;
  double t453;
  double t500;
  double t503;
  double t492;
  double t494;
  double t499;
  double t471;
  double t478;
  double t479;
  double t523;
  double t525;
  double t527;
  double t519;
  double t520;
  double t521;
  double t480;
  double t501;
  double t502;
  double t522;
  double t530;
  double t532;
  double t544;
  double t547;
  double t548;
  double t552;
  double t553;
  double t554;
  double t448;
  double t504;
  double t511;
  double t585;
  double t587;
  double t588;
  double t589;
  double t591;
  double t594;
  double t595;
  double t596;
  double t598;
  double t599;
  double t555;
  double t562;
  double t566;
  double t603;
  double t604;
  double t517;
  double t533;
  double t534;
  double t567;
  double t568;
  double t570;
  double t540;
  double t550;
  double t551;
  t432 = Cos(var1[3]);
  t458 = Cos(var1[5]);
  t473 = Sin(var1[3]);
  t465 = Sin(var1[4]);
  t475 = Sin(var1[5]);
  t444 = Cos(var1[4]);
  t447 = Cos(var1[7]);
  t453 = Cos(var1[6]);
  t500 = Sin(var1[6]);
  t503 = Sin(var1[7]);
  t492 = -1.*t458*t473;
  t494 = t432*t465*t475;
  t499 = t492 + t494;
  t471 = t432*t458*t465;
  t478 = t473*t475;
  t479 = t471 + t478;
  t523 = t432*t458;
  t525 = t473*t465*t475;
  t527 = t523 + t525;
  t519 = t458*t473*t465;
  t520 = -1.*t432*t475;
  t521 = t519 + t520;
  t480 = t453*t479;
  t501 = -1.*t499*t500;
  t502 = t480 + t501;
  t522 = t453*t521;
  t530 = -1.*t527*t500;
  t532 = t522 + t530;
  t544 = t444*t458*t453;
  t547 = -1.*t444*t475*t500;
  t548 = t544 + t547;
  t552 = t453*t499;
  t553 = t479*t500;
  t554 = t552 + t553;
  t448 = t432*t444*t447;
  t504 = -1.*t502*t503;
  t511 = t448 + t504;
  t585 = -1.*t447;
  t587 = 1. + t585;
  t588 = 0.183*t587;
  t589 = 0. + t588;
  t591 = -1.*t453;
  t594 = 1. + t591;
  t595 = -0.047*t594;
  t596 = 0. + t595;
  t598 = 0.047*t500;
  t599 = 0. + t598;
  t555 = t453*t527;
  t562 = t521*t500;
  t566 = t555 + t562;
  t603 = 0.183*t503;
  t604 = 0. + t603;
  t517 = t444*t447*t473;
  t533 = -1.*t532*t503;
  t534 = t517 + t533;
  t567 = t444*t453*t475;
  t568 = t444*t458*t500;
  t570 = t567 + t568;
  t540 = -1.*t447*t465;
  t550 = -1.*t548*t503;
  t551 = t540 + t550;
  p_output1[0]=t511;
  p_output1[1]=t534;
  p_output1[2]=t551;
  p_output1[3]=0.;
  p_output1[4]=t554;
  p_output1[5]=t566;
  p_output1[6]=t570;
  p_output1[7]=0.;
  p_output1[8]=t447*t502 + t432*t444*t503;
  p_output1[9]=t444*t473*t503 + t447*t532;
  p_output1[10]=-1.*t465*t503 + t447*t548;
  p_output1[11]=0.;
  p_output1[12]=0. + 0.183*t511 - 0.13205*t554 + t432*t444*t589 + t499*t596 + t479*t599 + t502*t604 + var1[0];
  p_output1[13]=0. + 0.183*t534 - 0.13205*t566 + t444*t473*t589 + t527*t596 + t521*t599 + t532*t604 + var1[1];
  p_output1[14]=0. + 0.183*t551 - 0.13205*t570 - 1.*t465*t589 + t444*t475*t596 + t444*t458*t599 + t548*t604 + var1[2];
  p_output1[15]=1.;
}



void H_FR_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
