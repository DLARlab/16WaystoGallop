/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:18 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FR_thigh_joint_src.h"

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
  double t378;
  double t396;
  double t398;
  double t393;
  double t399;
  double t382;
  double t397;
  double t400;
  double t401;
  double t403;
  double t405;
  double t407;
  double t408;
  double t346;
  double t350;
  double t421;
  double t422;
  double t423;
  double t424;
  double t355;
  double t373;
  double t375;
  double t376;
  double t383;
  double t387;
  double t388;
  double t389;
  double t409;
  double t413;
  double t434;
  double t437;
  double t438;
  double t440;
  double t441;
  double t442;
  double t425;
  double t426;
  double t448;
  double t449;
  double t450;
  double t467;
  double t469;
  double t470;
  double t486;
  double t487;
  double t488;
  double t505;
  double t511;
  double t514;
  double t523;
  double t524;
  double t525;
  double t534;
  double t535;
  double t536;
  double t540;
  double t541;
  double t542;
  double t415;
  double t549;
  double t550;
  double t560;
  double t561;
  double t562;
  double t569;
  double t570;
  double t545;
  double t575;
  double t576;
  double t577;
  double t582;
  double t583;
  double t556;
  double t557;
  double t558;
  double t559;
  double t563;
  double t564;
  double t579;
  double t580;
  t378 = Sin(var1[3]);
  t396 = Cos(var1[5]);
  t398 = Sin(var1[4]);
  t393 = Cos(var1[3]);
  t399 = Sin(var1[5]);
  t382 = Cos(var1[6]);
  t397 = -1.*t393*t396;
  t400 = -1.*t378*t398*t399;
  t401 = t397 + t400;
  t403 = -1.*t396*t378*t398;
  t405 = t393*t399;
  t407 = t403 + t405;
  t408 = Sin(var1[6]);
  t346 = Cos(var1[4]);
  t350 = Cos(var1[7]);
  t421 = t382*t407;
  t422 = -1.*t401*t408;
  t423 = t421 + t422;
  t424 = Sin(var1[7]);
  t355 = -1.*t350;
  t373 = 1. + t355;
  t375 = 0.183*t373;
  t376 = 0. + t375;
  t383 = -1.*t382;
  t387 = 1. + t383;
  t388 = -0.047*t387;
  t389 = 0. + t388;
  t409 = 0.047*t408;
  t413 = 0. + t409;
  t434 = -1.*t396*t378;
  t437 = t393*t398*t399;
  t438 = t434 + t437;
  t440 = t393*t396*t398;
  t441 = t378*t399;
  t442 = t440 + t441;
  t425 = 0.183*t424;
  t426 = 0. + t425;
  t448 = t382*t442;
  t449 = -1.*t438*t408;
  t450 = t448 + t449;
  t467 = t393*t346*t396*t382;
  t469 = -1.*t393*t346*t399*t408;
  t470 = t467 + t469;
  t486 = t346*t396*t382*t378;
  t487 = -1.*t346*t378*t399*t408;
  t488 = t486 + t487;
  t505 = -1.*t396*t382*t398;
  t511 = t398*t399*t408;
  t514 = t505 + t511;
  t523 = t396*t378;
  t524 = -1.*t393*t398*t399;
  t525 = t523 + t524;
  t534 = t382*t525;
  t535 = -1.*t442*t408;
  t536 = t534 + t535;
  t540 = t396*t378*t398;
  t541 = -1.*t393*t399;
  t542 = t540 + t541;
  t415 = t382*t401;
  t549 = -1.*t542*t408;
  t550 = t415 + t549;
  t560 = -1.*t346*t382*t399;
  t561 = -1.*t346*t396*t408;
  t562 = t560 + t561;
  t569 = -1.*t382*t438;
  t570 = t569 + t535;
  t545 = t382*t542;
  t575 = t393*t396;
  t576 = t378*t398*t399;
  t577 = t575 + t576;
  t582 = -1.*t382*t577;
  t583 = t582 + t549;
  t556 = t346*t396*t382;
  t557 = -1.*t346*t399*t408;
  t558 = t556 + t557;
  t559 = -0.13205*t558;
  t563 = t562*t426;
  t564 = -0.183*t562*t424;
  t579 = -1.*t577*t408;
  t580 = t545 + t579;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-1.*t346*t376*t378 + t389*t401 + t407*t413 - 0.13205*(t407*t408 + t415) + 0.183*(-1.*t346*t350*t378 - 1.*t423*t424) + t423*t426;
  p_output1[10]=t346*t376*t393 + t389*t438 + t413*t442 - 0.13205*(t382*t438 + t408*t442) + t426*t450 + 0.183*(t346*t350*t393 - 1.*t424*t450);
  p_output1[11]=0;
  p_output1[12]=-1.*t376*t393*t398 + t346*t389*t393*t399 - 0.13205*(t346*t382*t393*t399 + t346*t393*t396*t408) + t346*t393*t396*t413 + t426*t470 + 0.183*(-1.*t350*t393*t398 - 1.*t424*t470);
  p_output1[13]=-1.*t376*t378*t398 + t346*t378*t389*t399 - 0.13205*(t346*t378*t382*t399 + t346*t378*t396*t408) + t346*t378*t396*t413 + t426*t488 + 0.183*(-1.*t350*t378*t398 - 1.*t424*t488);
  p_output1[14]=-1.*t346*t376 - 1.*t389*t398*t399 - 0.13205*(-1.*t382*t398*t399 - 1.*t396*t398*t408) - 1.*t396*t398*t413 + t426*t514 + 0.183*(-1.*t346*t350 - 1.*t424*t514);
  p_output1[15]=t389*t442 + t413*t525 - 0.13205*(t448 + t408*t525) - 0.183*t424*t536 + t426*t536;
  p_output1[16]=t401*t413 + t389*t542 - 0.13205*(t401*t408 + t545) - 0.183*t424*t550 + t426*t550;
  p_output1[17]=t346*t389*t396 - 1.*t346*t399*t413 + t559 + t563 + t564;
  p_output1[18]=-0.047*t408*t438 + 0.047*t382*t442 - 0.13205*t450 - 0.183*t424*t570 + t426*t570;
  p_output1[19]=0.047*t382*t542 - 0.047*t408*t577 - 0.13205*t580 - 0.183*t424*t583 + t426*t583;
  p_output1[20]=0.047*t346*t382*t396 - 0.047*t346*t399*t408 + t559 + t563 + t564;
  p_output1[21]=0.183*t346*t393*t424 + 0.183*t350*t450 + 0.183*(-1.*t346*t393*t424 - 1.*t350*t450);
  p_output1[22]=0.183*t346*t378*t424 + 0.183*t350*t580 + 0.183*(-1.*t346*t378*t424 - 1.*t350*t580);
  p_output1[23]=-0.183*t398*t424 + 0.183*t350*t558 + 0.183*(t398*t424 - 1.*t350*t558);
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=0;
  p_output1[28]=0;
  p_output1[29]=0;
  p_output1[30]=0;
  p_output1[31]=0;
  p_output1[32]=0;
  p_output1[33]=0;
  p_output1[34]=0;
  p_output1[35]=0;
  p_output1[36]=0;
  p_output1[37]=0;
  p_output1[38]=0;
  p_output1[39]=0;
  p_output1[40]=0;
  p_output1[41]=0;
  p_output1[42]=0;
  p_output1[43]=0;
  p_output1[44]=0;
  p_output1[45]=0;
  p_output1[46]=0;
  p_output1[47]=0;
  p_output1[48]=0;
  p_output1[49]=0;
  p_output1[50]=0;
  p_output1[51]=0;
  p_output1[52]=0;
  p_output1[53]=0;
}



void Jp_FR_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
