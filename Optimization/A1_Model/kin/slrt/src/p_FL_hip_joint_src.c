/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:25 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FL_hip_joint_src.h"

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
  double t1385;
  double t1419;
  double t1424;
  double t1422;
  double t1428;
  double t1406;
  double t1423;
  double t1429;
  double t1431;
  double t1433;
  double t1438;
  double t1439;
  double t1440;
  double t1386;
  double t1407;
  double t1418;
  double t1434;
  double t1435;
  double t1436;
  double t1437;
  double t1449;
  double t1450;
  double t1451;
  double t1453;
  double t1454;
  double t1455;
  t1385 = Cos(var1[3]);
  t1419 = Cos(var1[5]);
  t1424 = Sin(var1[3]);
  t1422 = Sin(var1[4]);
  t1428 = Sin(var1[5]);
  t1406 = Sin(var1[9]);
  t1423 = t1385*t1419*t1422;
  t1429 = t1424*t1428;
  t1431 = t1423 + t1429;
  t1433 = Cos(var1[9]);
  t1438 = -1.*t1419*t1424;
  t1439 = t1385*t1422*t1428;
  t1440 = t1438 + t1439;
  t1386 = Cos(var1[4]);
  t1407 = -0.047*t1406;
  t1418 = 0. + t1407;
  t1434 = -1.*t1433;
  t1435 = 1. + t1434;
  t1436 = 0.047*t1435;
  t1437 = 0. + t1436;
  t1449 = t1419*t1424*t1422;
  t1450 = -1.*t1385*t1428;
  t1451 = t1449 + t1450;
  t1453 = t1385*t1419;
  t1454 = t1424*t1422*t1428;
  t1455 = t1453 + t1454;
  p_output1[0]=0. + 0.183*t1385*t1386 + t1418*t1431 + t1437*t1440 + 0.047*(t1406*t1431 + t1433*t1440) + var1[0];
  p_output1[1]=0. + 0.183*t1386*t1424 + t1418*t1451 + t1437*t1455 + 0.047*(t1406*t1451 + t1433*t1455) + var1[1];
  p_output1[2]=0. + t1386*t1418*t1419 - 0.183*t1422 + 0.047*(t1386*t1406*t1419 + t1386*t1428*t1433) + t1386*t1428*t1437 + var1[2];
}



void p_FL_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
