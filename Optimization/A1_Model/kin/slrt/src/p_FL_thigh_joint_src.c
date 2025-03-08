/*
 * Automatically Generated from Mathematica.
 * Mon 22 Mar 2021 16:01:29 GMT-04:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "p_FL_thigh_joint_src.h"

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
  double t1634;
  double t1645;
  double t1648;
  double t1646;
  double t1649;
  double t1640;
  double t1647;
  double t1651;
  double t1652;
  double t1656;
  double t1661;
  double t1662;
  double t1663;
  double t1611;
  double t1635;
  double t1671;
  double t1674;
  double t1675;
  double t1678;
  double t1625;
  double t1627;
  double t1628;
  double t1631;
  double t1641;
  double t1643;
  double t1657;
  double t1658;
  double t1659;
  double t1660;
  double t1686;
  double t1687;
  double t1688;
  double t1690;
  double t1691;
  double t1694;
  double t1672;
  double t1673;
  double t1700;
  double t1701;
  double t1702;
  double t1716;
  double t1717;
  double t1718;
  t1634 = Cos(var1[3]);
  t1645 = Cos(var1[5]);
  t1648 = Sin(var1[3]);
  t1646 = Sin(var1[4]);
  t1649 = Sin(var1[5]);
  t1640 = Sin(var1[9]);
  t1647 = t1634*t1645*t1646;
  t1651 = t1648*t1649;
  t1652 = t1647 + t1651;
  t1656 = Cos(var1[9]);
  t1661 = -1.*t1645*t1648;
  t1662 = t1634*t1646*t1649;
  t1663 = t1661 + t1662;
  t1611 = Cos(var1[10]);
  t1635 = Cos(var1[4]);
  t1671 = Sin(var1[10]);
  t1674 = t1656*t1652;
  t1675 = -1.*t1640*t1663;
  t1678 = t1674 + t1675;
  t1625 = -1.*t1611;
  t1627 = 1. + t1625;
  t1628 = 0.183*t1627;
  t1631 = 0. + t1628;
  t1641 = -0.047*t1640;
  t1643 = 0. + t1641;
  t1657 = -1.*t1656;
  t1658 = 1. + t1657;
  t1659 = 0.047*t1658;
  t1660 = 0. + t1659;
  t1686 = t1645*t1648*t1646;
  t1687 = -1.*t1634*t1649;
  t1688 = t1686 + t1687;
  t1690 = t1634*t1645;
  t1691 = t1648*t1646*t1649;
  t1694 = t1690 + t1691;
  t1672 = 0.183*t1671;
  t1673 = 0. + t1672;
  t1700 = t1656*t1688;
  t1701 = -1.*t1640*t1694;
  t1702 = t1700 + t1701;
  t1716 = t1656*t1635*t1645;
  t1717 = -1.*t1635*t1640*t1649;
  t1718 = t1716 + t1717;
  p_output1[0]=0. + t1631*t1634*t1635 + t1643*t1652 + t1660*t1663 + 0.13205*(t1640*t1652 + t1656*t1663) + t1673*t1678 + 0.183*(t1611*t1634*t1635 - 1.*t1671*t1678) + var1[0];
  p_output1[1]=0. + t1631*t1635*t1648 + t1643*t1688 + t1660*t1694 + 0.13205*(t1640*t1688 + t1656*t1694) + t1673*t1702 + 0.183*(t1611*t1635*t1648 - 1.*t1671*t1702) + var1[1];
  p_output1[2]=0. + t1635*t1643*t1645 - 1.*t1631*t1646 + 0.13205*(t1635*t1640*t1645 + t1635*t1649*t1656) + t1635*t1649*t1660 + t1673*t1718 + 0.183*(-1.*t1611*t1646 - 1.*t1671*t1718) + var1[2];
}



void p_FL_thigh_joint_src(double *p_output1, const double *var1)
{
  /* Call Subroutines */
  output1(p_output1, var1);

}
