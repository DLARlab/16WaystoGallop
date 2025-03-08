/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:26 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_FL_hip_joint_src.h"

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
  double t1422;
  double t1462;
  double t1458;
  double t1460;
  double t1463;
  double t1448;
  double t1461;
  double t1464;
  double t1465;
  double t1467;
  double t1474;
  double t1475;
  double t1476;
  double t1407;
  double t1449;
  double t1454;
  double t1468;
  double t1469;
  double t1470;
  double t1472;
  double t1484;
  double t1485;
  double t1486;
  double t1488;
  double t1489;
  double t1490;
  double t1526;
  double t1528;
  double t1530;
  double t1537;
  double t1538;
  double t1539;
  double t1532;
  double t1543;
  double t1562;
  double t1563;
  double t1564;
  double t1550;
  double t1551;
  double t1552;
  double t1553;
  t1422 = Sin(var1[3]);
  t1462 = Cos(var1[3]);
  t1458 = Cos(var1[5]);
  t1460 = Sin(var1[4]);
  t1463 = Sin(var1[5]);
  t1448 = Sin(var1[9]);
  t1461 = -1.*t1458*t1422*t1460;
  t1464 = t1462*t1463;
  t1465 = t1461 + t1464;
  t1467 = Cos(var1[9]);
  t1474 = -1.*t1462*t1458;
  t1475 = -1.*t1422*t1460*t1463;
  t1476 = t1474 + t1475;
  t1407 = Cos(var1[4]);
  t1449 = -0.047*t1448;
  t1454 = 0. + t1449;
  t1468 = -1.*t1467;
  t1469 = 1. + t1468;
  t1470 = 0.047*t1469;
  t1472 = 0. + t1470;
  t1484 = t1462*t1458*t1460;
  t1485 = t1422*t1463;
  t1486 = t1484 + t1485;
  t1488 = -1.*t1458*t1422;
  t1489 = t1462*t1460*t1463;
  t1490 = t1488 + t1489;
  t1526 = t1458*t1422;
  t1528 = -1.*t1462*t1460*t1463;
  t1530 = t1526 + t1528;
  t1537 = t1458*t1422*t1460;
  t1538 = -1.*t1462*t1463;
  t1539 = t1537 + t1538;
  t1532 = t1467*t1486;
  t1543 = t1467*t1539;
  t1562 = t1462*t1458;
  t1563 = t1422*t1460*t1463;
  t1564 = t1562 + t1563;
  t1550 = t1467*t1407*t1458;
  t1551 = -1.*t1407*t1448*t1463;
  t1552 = t1550 + t1551;
  t1553 = 0.047*t1552;
  p_output1[0]=1.;
  p_output1[1]=0;
  p_output1[2]=0;
  p_output1[3]=0;
  p_output1[4]=1.;
  p_output1[5]=0;
  p_output1[6]=0;
  p_output1[7]=0;
  p_output1[8]=1.;
  p_output1[9]=-0.183*t1407*t1422 + t1454*t1465 + t1472*t1476 + 0.047*(t1448*t1465 + t1467*t1476);
  p_output1[10]=0.183*t1407*t1462 + t1454*t1486 + t1472*t1490 + 0.047*(t1448*t1486 + t1467*t1490);
  p_output1[11]=0;
  p_output1[12]=t1407*t1454*t1458*t1462 - 0.183*t1460*t1462 + 0.047*(t1407*t1448*t1458*t1462 + t1407*t1462*t1463*t1467) + t1407*t1462*t1463*t1472;
  p_output1[13]=t1407*t1422*t1454*t1458 - 0.183*t1422*t1460 + 0.047*(t1407*t1422*t1448*t1458 + t1407*t1422*t1463*t1467) + t1407*t1422*t1463*t1472;
  p_output1[14]=-0.183*t1407 - 1.*t1454*t1458*t1460 + 0.047*(-1.*t1448*t1458*t1460 - 1.*t1460*t1463*t1467) - 1.*t1460*t1463*t1472;
  p_output1[15]=t1472*t1486 + t1454*t1530 + 0.047*(t1448*t1530 + t1532);
  p_output1[16]=t1454*t1476 + t1472*t1539 + 0.047*(t1448*t1476 + t1543);
  p_output1[17]=-1.*t1407*t1454*t1463 + t1407*t1458*t1472 + t1553;
  p_output1[18]=0;
  p_output1[19]=0;
  p_output1[20]=0;
  p_output1[21]=0;
  p_output1[22]=0;
  p_output1[23]=0;
  p_output1[24]=0;
  p_output1[25]=0;
  p_output1[26]=0;
  p_output1[27]=-0.047*t1467*t1486 + 0.047*t1448*t1490 + 0.047*(-1.*t1448*t1490 + t1532);
  p_output1[28]=-0.047*t1467*t1539 + 0.047*t1448*t1564 + 0.047*(t1543 - 1.*t1448*t1564);
  p_output1[29]=0.047*t1407*t1448*t1463 - 0.047*t1407*t1458*t1467 + t1553;
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



void Jp_FL_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
