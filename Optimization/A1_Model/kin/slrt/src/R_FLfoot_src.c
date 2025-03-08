/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:02:06 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_FLfoot_src.h"

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
  double t1343;
  double t1372;
  double t1378;
  double t1375;
  double t1381;
  double t1368;
  double t1345;
  double t1363;
  double t1369;
  double t1376;
  double t1382;
  double t1387;
  double t1388;
  double t1390;
  double t1391;
  double t1392;
  double t1393;
  double t1394;
  double t1399;
  double t1320;
  double t1409;
  double t6427;
  double t6470;
  double t6475;
  double t6493;
  double t6514;
  double t6519;
  double t6523;
  double t6524;
  double t6530;
  double t6548;
  double t6549;
  double t6550;
  double t1367;
  double t1403;
  double t1404;
  double t1415;
  double t1416;
  double t2035;
  double t6423;
  double t6532;
  double t6538;
  double t6540;
  double t6543;
  double t6544;
  double t6547;
  double t6551;
  double t6554;
  double t6556;
  double t6557;
  double t6565;
  t1343 = Cos(var1[3]);
  t1372 = Cos(var1[5]);
  t1378 = Sin(var1[3]);
  t1375 = Sin(var1[4]);
  t1381 = Sin(var1[5]);
  t1368 = Cos(var1[10]);
  t1345 = Cos(var1[4]);
  t1363 = Sin(var1[10]);
  t1369 = Cos(var1[9]);
  t1376 = t1343*t1372*t1375;
  t1382 = t1378*t1381;
  t1387 = t1376 + t1382;
  t1388 = t1369*t1387;
  t1390 = Sin(var1[9]);
  t1391 = -1.*t1372*t1378;
  t1392 = t1343*t1375*t1381;
  t1393 = t1391 + t1392;
  t1394 = -1.*t1390*t1393;
  t1399 = t1388 + t1394;
  t1320 = Sin(var1[11]);
  t1409 = Cos(var1[11]);
  t6427 = t1372*t1378*t1375;
  t6470 = -1.*t1343*t1381;
  t6475 = t6427 + t6470;
  t6493 = t1369*t6475;
  t6514 = t1343*t1372;
  t6519 = t1378*t1375*t1381;
  t6523 = t6514 + t6519;
  t6524 = -1.*t1390*t6523;
  t6530 = t6493 + t6524;
  t6548 = t1369*t1345*t1372;
  t6549 = -1.*t1345*t1390*t1381;
  t6550 = t6548 + t6549;
  t1367 = t1343*t1345*t1363;
  t1403 = t1368*t1399;
  t1404 = t1367 + t1403;
  t1415 = t1368*t1343*t1345;
  t1416 = -1.*t1363*t1399;
  t2035 = t1415 + t1416;
  t6423 = t1345*t1363*t1378;
  t6532 = t1368*t6530;
  t6538 = t6423 + t6532;
  t6540 = t1368*t1345*t1378;
  t6543 = -1.*t1363*t6530;
  t6544 = t6540 + t6543;
  t6547 = -1.*t1363*t1375;
  t6551 = t1368*t6550;
  t6554 = t6547 + t6551;
  t6556 = -1.*t1368*t1375;
  t6557 = -1.*t1363*t6550;
  t6565 = t6556 + t6557;
  p_output1[0]=-1.*t1320*t1404 + t1409*t2035;
  p_output1[1]=-1.*t1320*t6538 + t1409*t6544;
  p_output1[2]=-1.*t1320*t6554 + t1409*t6565;
  p_output1[3]=t1387*t1390 + t1369*t1393;
  p_output1[4]=t1390*t6475 + t1369*t6523;
  p_output1[5]=t1345*t1369*t1381 + t1345*t1372*t1390;
  p_output1[6]=t1404*t1409 + t1320*t2035;
  p_output1[7]=t1409*t6538 + t1320*t6544;
  p_output1[8]=t1409*t6554 + t1320*t6565;
}



void R_FLfoot_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
