/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:27 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "H_FL_hip_joint_src.h"

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
  double t1483;
  double t1481;
  double t1500;
  double t1496;
  double t1510;
  double t1514;
  double t1507;
  double t1521;
  double t1511;
  double t1516;
  double t1517;
  double t1522;
  double t1523;
  double t1524;
  double t1530;
  double t1531;
  double t1533;
  double t1535;
  double t1536;
  double t1537;
  double t1520;
  double t1525;
  double t1526;
  double t1563;
  double t1565;
  double t1568;
  double t1569;
  double t1570;
  double t1571;
  double t1534;
  double t1539;
  double t1541;
  double t1542;
  double t1544;
  double t1547;
  t1483 = Cos(var1[4]);
  t1481 = Cos(var1[3]);
  t1500 = Sin(var1[4]);
  t1496 = Sin(var1[3]);
  t1510 = Cos(var1[5]);
  t1514 = Sin(var1[5]);
  t1507 = Sin(var1[9]);
  t1521 = Cos(var1[9]);
  t1511 = t1481*t1510*t1500;
  t1516 = t1496*t1514;
  t1517 = t1511 + t1516;
  t1522 = -1.*t1510*t1496;
  t1523 = t1481*t1500*t1514;
  t1524 = t1522 + t1523;
  t1530 = t1510*t1496*t1500;
  t1531 = -1.*t1481*t1514;
  t1533 = t1530 + t1531;
  t1535 = t1481*t1510;
  t1536 = t1496*t1500*t1514;
  t1537 = t1535 + t1536;
  t1520 = t1507*t1517;
  t1525 = t1521*t1524;
  t1526 = t1520 + t1525;
  t1563 = -0.047*t1507;
  t1565 = 0. + t1563;
  t1568 = -1.*t1521;
  t1569 = 1. + t1568;
  t1570 = 0.047*t1569;
  t1571 = 0. + t1570;
  t1534 = t1507*t1533;
  t1539 = t1521*t1537;
  t1541 = t1534 + t1539;
  t1542 = t1483*t1510*t1507;
  t1544 = t1521*t1483*t1514;
  t1547 = t1542 + t1544;
  p_output1[0]=t1481*t1483;
  p_output1[1]=t1483*t1496;
  p_output1[2]=-1.*t1500;
  p_output1[3]=0.;
  p_output1[4]=t1526;
  p_output1[5]=t1541;
  p_output1[6]=t1547;
  p_output1[7]=0.;
  p_output1[8]=t1517*t1521 - 1.*t1507*t1524;
  p_output1[9]=t1521*t1533 - 1.*t1507*t1537;
  p_output1[10]=-1.*t1483*t1507*t1514 + t1483*t1510*t1521;
  p_output1[11]=0.;
  p_output1[12]=0. + 0.183*t1481*t1483 + 0.047*t1526 + t1517*t1565 + t1524*t1571 + var1[0];
  p_output1[13]=0. + 0.183*t1483*t1496 + 0.047*t1541 + t1533*t1565 + t1537*t1571 + var1[1];
  p_output1[14]=0. - 0.183*t1500 + 0.047*t1547 + t1483*t1510*t1565 + t1483*t1514*t1571 + var1[2];
  p_output1[15]=1.;
}



void H_FL_hip_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
